"""Isaac Lab 2.1 environment for the SimpleFlight Direct-CTBR tasks."""

from __future__ import annotations

import math

import torch
from pxr import Gf, UsdGeom, UsdPhysics

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from .actuation import CTBRActuator, POLICY_DT, load_crazyflie_params, rigid_body_properties
from .task import FlightTask, episode_duration_s, observation_dim


@configclass
class FlightEnvCfg(DirectRLEnvCfg):
    airframe: str = "crazyflie"
    task: str = "hover"
    evaluation: bool = False
    eval_profile: str = "standard"  # Also selects the Hover reset range during training.

    decimation = 1
    episode_length_s = 5.0
    action_space = 4
    observation_space = 15
    state_space = 19
    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=POLICY_DT,
        render_interval=1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0, dynamic_friction=1.0, restitution=0.0
        ),
    )
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1024, env_spacing=4.0, replicate_physics=True)


class FlightEnv(DirectRLEnv):
    """One isolated quadrotor per scene instance, with airframe-specific timing."""

    cfg: FlightEnvCfg

    def __init__(self, cfg: FlightEnvCfg, render_mode: str | None = None):
        cfg = cfg.copy()
        self._rm27 = cfg.airframe == "rm27"
        if self._rm27:
            from .rm27 import load_rm27_profile

            self._params = load_rm27_profile()
            cfg.sim.dt = self._params.physics_dt_s
            cfg.decimation = round(self._params.policy_dt_s / cfg.sim.dt)
            cfg.sim.render_interval = cfg.decimation
            self.mass_kg = self._params.mass_kg
            self.inertia_kg_m2 = self._params.inertia_kg_m2
            self._cylinder_size = (self._params.outer_width_m / 2, self._params.outer_height_m)
        elif cfg.airframe == "crazyflie":
            if cfg.decimation != 1 or cfg.sim.dt != POLICY_DT:
                raise ValueError(f"SimpleFlight expects one physics step every {POLICY_DT} s")
            self._params = load_crazyflie_params()
            self.mass_kg, self.inertia_kg_m2 = rigid_body_properties(self._params)
            self._cylinder_size = (0.060, 0.025)
        else:
            raise ValueError(f"Unknown airframe {cfg.airframe!r}")
        self.legacy_flight = None
        if cfg.evaluation and cfg.eval_profile == "legacy" and self._rm27:
            from .rm27_legacy import load_legacy_flight

            self.legacy_flight = load_legacy_flight(cfg.task)
            if cfg.scene.num_envs != len(self.legacy_flight.position):
                raise ValueError("The RM27 legacy suite requires all of its fixed cases")
        cfg.observation_space = observation_dim(cfg.task) + (3 if self._rm27 else 0)
        cfg.state_space = cfg.observation_space + 4
        cfg.episode_length_s = (self.legacy_flight.horizon_s if self.legacy_flight is not None
                                else episode_duration_s(cfg.task))
        super().__init__(cfg, render_mode)
        self._task = FlightTask(
            cfg.task, self.num_envs, self.device, self.step_dt,
            evaluation=cfg.evaluation, eval_profile=cfg.eval_profile, rm27=self._rm27,
            legacy_flight=self.legacy_flight,
        )
        self._env_ids = torch.arange(self.num_envs, device=self.device)
        actuator_type = CTBRActuator
        if self._rm27:
            from .rm27 import RM27Actuator

            actuator_type = RM27Actuator
        self._actuator = actuator_type(self._params, self.num_envs, self.device, math.hypot(*cfg.sim.gravity))
        self.ctbr_command = torch.zeros((self.num_envs, 4), device=self.device)
        self._action_delta = torch.zeros(self.num_envs, device=self.device)
        self.last_step: dict[str, torch.Tensor] = {}

    @property
    def hover_logit(self) -> float:
        """Collective policy input that balances gravity when level and still."""
        return self._actuator.hover_logit

    def state(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Position relative to environment origin, wxyz attitude, world m/s, body rad/s."""
        data = self._robot.data
        return data.root_pos_w - self.scene.env_origins, data.root_quat_w, data.root_lin_vel_w, data.root_ang_vel_b

    def reference(self) -> torch.Tensor:
        """Current target position relative to each environment origin."""
        return self._task.reference(self.episode_length_buf, lookahead=False)[:, 0]

    def _setup_scene(self):
        radius, height = self._cylinder_size
        # RM27 uses an envelope proxy for floor detection. The task stays in free flight.
        prim_name = "RM27" if self._rm27 else "Crazyflie"
        robot = RigidObjectCfg(
            prim_path=f"{self.scene.env_prim_paths[0]}/{prim_name}",
            spawn=sim_utils.CylinderCfg(
                radius=radius,
                height=height,
                mass_props=sim_utils.MassPropertiesCfg(mass=self.mass_kg),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    linear_damping=0.0, angular_damping=0.0, enable_gyroscopic_forces=True, sleep_threshold=0.0,
                ),
                collision_props=sim_utils.CollisionPropertiesCfg(),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.5)),
        )
        # The source exists before cloning, but the physics view must match all instances.
        prim = robot.spawn.func(robot.prim_path, robot.spawn, translation=robot.init_state.pos)
        mass_api = UsdPhysics.MassAPI(prim)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*self.inertia_kg_m2))
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0))
        mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1.0))
        robot.prim_path = f"{self.scene.env_regex_ns}/{prim_name}"
        robot.spawn = None
        self._robot = RigidObject(robot)
        self.scene.rigid_objects[self.cfg.airframe] = self._robot

        # Use the same infinite Z plane collider as upstream, without fetching
        # the visual grid asset from Nucleus.
        ground = UsdGeom.Plane.Define(prim.GetStage(), "/World/ground")
        ground.CreateAxisAttr("Z")
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=["/World/ground"])

    def _pre_physics_step(self, actions: torch.Tensor):
        command = actions.tanh()
        self._action_delta = torch.linalg.vector_norm(command - self.ctbr_command, dim=-1)
        self.ctbr_command = command
        if self._rm27:
            self._actuator.set_ctbr(self._actuator.decode_command(command), self._robot.data.root_ang_vel_b)

    def _apply_action(self):
        if self._rm27:
            force_b, torque_b = self._actuator.step_wrench()
        else:
            force_b, torque_b = self._actuator.update(self.ctbr_command, self._robot.data.root_ang_vel_b)
        self._apply_wrench(force_b, torque_b)

    def _apply_wrench(self, force_b: torch.Tensor, torque_b: torch.Tensor):
        # Same PhysX call as RigidObject.write_data_to_sim. With an explicit wrench
        # every tick we can skip the wrapper's nonzero test and CUDA host sync.
        self._robot.root_physx_view.apply_forces_and_torques_at_position(
            force_data=force_b, torque_data=torque_b, position_data=None, indices=self._env_ids, is_global=False
        )

    def _get_observations(self) -> dict[str, torch.Tensor]:
        pos, quat, velocity, body_rate = self.state()
        return self._task.observe(pos, quat, velocity, self.episode_length_buf, body_rate if self._rm27 else None)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        pos, quat, velocity, body_rate = self.state()
        self._step_reward, terminated, self.step_error_m = self._task.outcome(
            pos, quat, velocity, body_rate, self.episode_length_buf, self._action_delta
        )
        if self.cfg.evaluation:
            # Lab resets done instances before returning. Capture the actual final state.
            self.last_step = {
                "position": pos,
                "reference": self.reference().clone(),
                "quat": quat.clone(),
                "linear_velocity": velocity.clone(),
                "angular_velocity": body_rate.clone(),
                "ctbr_command": self.ctbr_command.clone(),
            }
            if self._rm27:
                self.last_step.update(
                    motor_thrust_N=self._actuator.motor_thrust_N.clone(),
                    motor_target_N=self._actuator.motor_command_N.clone(),
                    mixer_saturated=self._actuator.mixer_saturated[:, None].float(),
                )
            self.extras["flight"] = self.last_step
        time_out = self.episode_length_buf >= self.max_episode_length
        return terminated, time_out

    def _get_rewards(self) -> torch.Tensor:
        # DirectRLEnv calls _get_dones before _get_rewards on every step.
        return self._step_reward

    def _reset_idx(self, env_ids: torch.Tensor):
        super()._reset_idx(env_ids)
        self._actuator.reset(env_ids)
        self.ctbr_command[env_ids] = 0.0
        self._action_delta[env_ids] = 0.0
        pos, quat, velocity = self._task.reset(env_ids)
        pose = torch.cat((pos + self.scene.env_origins[env_ids], quat), dim=-1)
        self._robot.write_root_pose_to_sim(pose, env_ids)
        self._robot.write_root_velocity_to_sim(velocity, env_ids)
