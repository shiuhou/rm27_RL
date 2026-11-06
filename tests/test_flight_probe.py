"""Focused flight-task contracts; no optimizer update or duplicate full baseline suite."""
import json
import numpy as np
import pytest
from rm27_contact_rl.paths import ROOT
from rm27_contact_rl.flight_probe import FlightProbeEnv, fixed_cases, TASK_ID

@pytest.fixture(scope='module')
def pair():
    envs=[FlightProbeEnv(ROOT/f'configs/flight_probe/task_{a}.json') for a in ('a','b')]
    yield envs
    for env in envs:env.close()

def test_reference_observation_and_single_cost_factor(pair):
    a,b=pair
    ca,cb=a.rl_config.copy(),b.rl_config.copy()
    assert ca.pop('lambda_control')==0 and cb.pop('lambda_control')==1
    assert ca==cb
    options=dict(reference_index=1,initial_perturbation=np.zeros(12))
    oa,_=a.reset(seed=0,options=options);ob,_=b.reset(seed=0,options=options)
    np.testing.assert_array_equal(oa,ob)
    np.testing.assert_allclose(oa[:3],[.2,0,0],atol=1e-7)
    assert oa.shape==(26,) and len(fixed_cases(a))==23
    before=a._observation().copy();saved=a.plant.data.act.copy()
    a.plant.data.act[:4]*=.5
    np.testing.assert_array_equal(a._observation(),before)
    a.plant.data.act[:]=saved
    for _ in range(8):
        command=np.r_[a.plant.hover_force,[.1,-.1,.05]]
        xa=a.step(command);xb=b.step(command)
        np.testing.assert_array_equal(a.plant.data.qpos,b.plant.data.qpos)
        np.testing.assert_array_equal(xa[0],xb[0])
        assert xa[2:4]==xb[2:4]
        assert xb[1]-xa[1]==pytest.approx(xb[4]['reward_components']['control'],abs=1e-12)
        for key in xa[4]['reward_components']:
            if key!='control':assert xa[4]['reward_components'][key]==xb[4]['reward_components'][key]
        assert xa[4]['motor_force_intervals_N'].shape==(5,4)
    c=a.rl_config['control_cost'];m=a.evaluator.last_metrics
    expected=sum(min((m[key]/c[scale])**2,4) for key,scale in [('position_error_m','position_scale_m'),('speed_m_s','speed_scale_m_s'),('tilt_deg','tilt_scale_deg')])
    assert b._running_reward(m,m,command,command,.001)['control']==pytest.approx(-.001*expected)

def test_stability_window_and_any_contact_are_independent(pair):
    a,_=pair;a.reset(seed=0,options=dict(reference_index=0,initial_perturbation=np.zeros(12)))
    ev=a.evaluator
    assert ev.stable(ev.last_metrics)
    for key,threshold in [('position_error_m',.05),('speed_m_s',.1),('tilt_deg',5.),('body_rate_rad_s',.2)]:
        m=ev.last_metrics.copy();m[key]=np.nextafter(threshold,np.inf)
        assert not ev.stable(m)
    # Exercise the actual horizon branch with a known last-window history.
    # This is a clock/history unit fixture, not a claimed 4-second physical flight.
    ev.completed_intervals=3999;ev.elapsed_s=3.999;ev.stable_since=3.501
    a.step(np.r_[a.plant.hover_force,0,0,0])
    assert a._terminal_result['failure_modes']==['NOT_STABLE_AT_HORIZON']
    with pytest.raises(RuntimeError):a.step(np.zeros(4))
    a.reset(seed=0,options=dict(reference_index=0,initial_perturbation=np.zeros(12)))
    a.evaluator.completed_intervals=3999;a.evaluator.elapsed_s=3.999;a.evaluator.stable_since=3.5
    a.step(np.r_[a.plant.hover_force,0,0,0])
    assert a._terminal_result['passed']

def test_checkpoint_roundtrip_and_task_isolation(tmp_path):
    from stable_baselines3 import PPO
    from rm27_contact_rl.ppo import make_vec_env,_algorithm_kwargs,_save_checkpoint,_load_checkpoint,_model_state
    from rm27_contact_rl.provenance import provenance
    c=json.loads((ROOT/'configs/flight_probe/ppo_a.json').read_text())
    vec=make_vec_env(c);model=PPO('MlpPolicy',vec,seed=0,device='cpu',**_algorithm_kwargs(c));vec.reset()
    try:
        source=provenance(vec.venv.envs[0].unwrapped,algorithm='PPO_CPU')
        old=_model_state(model,vec);_save_checkpoint(tmp_path/'initial',model,vec,c,source,0)
        restored,v,_=_load_checkpoint(tmp_path/'initial',training=False,expected_task=TASK_ID)
        try:assert _model_state(restored,v)==old
        finally:v.close()
        with pytest.raises(ValueError,match='different task'):
            _load_checkpoint(tmp_path/'initial',training=False,expected_task='fixed_surface_contact_v1')
    finally:vec.close()


def test_recorded_flight_replay_uses_the_same_base_plant(pair, tmp_path, monkeypatch):
    import mujoco
    from rm27_contact_rl.stage1_replay import _episode, _restore_frame
    env = pair[0]
    env.reset(seed=0, options=dict(reference_index=1, initial_perturbation=np.zeros(12)))
    metadata = dict(rl=env.rl_config, private_model_path=str(env.plant.sources['private_scene_binary']))
    (tmp_path/'config.json').write_text(json.dumps(metadata))
    data = env.plant.data
    np.savez(tmp_path/'trajectory.npz', time_s=[0.], qpos=data.qpos[None], qvel=data.qvel[None],
             motor_state_N=data.act[None], motor_commands_N=data.ctrl[None],
             action_ctbr=env.previous_action_ctbr[None])
    def forbidden(*args, **kwargs):
        raise AssertionError('Recorded-state replay must not integrate physics')
    monkeypatch.setattr(mujoco, 'mj_step', forbidden)
    plant, trajectory = _episode(tmp_path)
    _restore_frame(plant, trajectory, 0)
    np.testing.assert_array_equal(plant.data.qpos, data.qpos)
    assert plant.cfg['task_id'] == 'fixed_surface_contact_v1'
    assert metadata['rl']['task_id'] == TASK_ID
    changed = json.loads((tmp_path/'config.json').read_text())
    changed['rl']['safety']['max_speed_m_s'] += 1
    (tmp_path/'config.json').write_text(json.dumps(changed))
    with pytest.raises(ValueError, match='differs from its base configuration'):
        _episode(tmp_path)

def test_real_contact_is_always_failure_in_flight_probe(pair):
    from rm27_contact_rl.flight_probe import FlightEvaluator
    env,_=pair
    env.reset(seed=0,options=dict(reference_index=0,initial_perturbation=np.zeros(12)))
    g=env.geometry;rotation=g['target_rotation']
    position=g['center']+(.001+g['radius'])*g['normal']-rotation@g['offset_body']
    env.plant.reset(position=position,rotation=rotation,velocity=-.2*g['normal'],omega_body=np.zeros(3))
    env.evaluator=FlightEvaluator(env.rl_config,env.plant,env.reference_position,env.hover_position)
    env.evaluator.initialize();assert not env.evaluator.done
    for _ in range(20):
        _,_,done,_,info=env.step(np.r_[env.plant.hover_force,0,0,0])
        if done:break
    assert done and not info['result']['passed']
    assert 'UNEXPECTED_CONTACT' in info['result']['failure_modes']
    assert info['result']['first_contact']['contacts']


def test_recorder_keeps_distinct_actor_fields_through_vector_autoreset(tmp_path):
    from rm27_contact_rl.ppo import make_vec_env
    from rm27_contact_rl.flight_probe_runner import Recorder
    from rm27_contact_rl.stage1_runner import SmokeBudget
    c=json.loads((ROOT/'configs/flight_probe/ppo_a.json').read_text())
    vec=make_vec_env(c);rec=Recorder(vec.venv.envs[0],tmp_path/'episodes',SmokeBudget(controls=2,substeps=10,wall_s=20));vec.venv.envs[0]=rec
    try:
        vec.reset();raw=rec.unwrapped;old_index=raw.reference_index
        raw.evaluator.completed_intervals=3999;raw.evaluator.elapsed_s=3.999;raw.evaluator.stable_since=None
        vec.step(np.array([[.1,.2,.3,.4]],np.float32))
        rec.annotate(gaussian_sample=np.array([.1,.2,1.3,.4]),actor_mean=np.array([.11,.22,.33,.44]),actor_std=np.ones(4)*.2)
        with np.load(tmp_path/'episodes/episode_00000/trajectory.npz') as a:
            np.testing.assert_array_equal(a['gaussian_sample'][-1],[.1,.2,1.3,.4])
            np.testing.assert_array_equal(a['actor_mean'][-1],[.11,.22,.33,.44])
            np.testing.assert_allclose(a['executed_action_normalized'][-1],[.1,.2,.3,.4])
            assert a['reference_index'][-1]==old_index and a['physics_substeps'][-1]==1
        assert rec.completed==1
    finally:vec.close()


def test_failure_tail_matches_explicit_absorbing_extension():
    from rm27_contact_rl.flight_probe import failure_tail_reward
    gamma=.999
    # Independently price each remaining 1ms interval on the PPO control grid.
    for end in [0,1,4,5,6,937,3999,4000]:
        start=0 if end==0 else ((end-1)//5)*5
        terminal_index=start//5
        explicit=-sum(.012*gamma**(interval//5-terminal_index) for interval in range(end,4000))
        folded=failure_tail_reward(start,end)
        assert folded==pytest.approx(explicit,abs=2e-12)
        rewards=np.linspace(-.01,-.02,terminal_index+1);rewards[-1]-=5
        short=np.dot(gamma**np.arange(len(rewards)),rewards)+gamma**terminal_index*folded
        extended=np.zeros(800);extended[:len(rewards)]=rewards
        for interval in range(end,4000):extended[interval//5]-=.012
        assert short==pytest.approx(np.dot(gamma**np.arange(800),extended),abs=2e-12)
    with pytest.raises(ValueError):failure_tail_reward(3,5)


def test_failure_tail_only_changes_terminal_reward_and_never_integrates_tail():
    from rm27_contact_rl.flight_probe import failure_tail_reward
    b=FlightProbeEnv(ROOT/'configs/flight_probe/task_b.json')
    c=FlightProbeEnv(ROOT/'configs/flight_probe/task_c.json')
    try:
        opts=dict(reference_index=0,initial_perturbation=np.zeros(12))
        b.reset(seed=0,options=opts);c.reset(seed=0,options=opts)
        controls=0
        # Actual original flight-volume safety termination under zero collective.
        while b._active:
            controls+=1;start=c.evaluator.completed_intervals
            xb=b.step(np.zeros(4));xc=c.step(np.zeros(4))
            np.testing.assert_array_equal(b.plant.snapshot(),c.plant.snapshot())
            np.testing.assert_array_equal(xb[0],xc[0]);assert xb[2:4]==xc[2:4]
            assert xb[4]['physics_substeps']==xc[4]['physics_substeps']
            delta=failure_tail_reward(start,c.evaluator.completed_intervals) if xc[2] else 0.
            assert xc[1]-xb[1]==pytest.approx(delta,abs=1e-12)
            if xc[2]:
                assert xc[4]['result']['failure_modes']==xb[4]['result']['failure_modes']==['FLIGHT_VOLUME_EXIT']
                assert xc[4]['result']['failure_tail']['simulated_tail_intervals']==0
                assert xc[4]['result']['total_reward']==pytest.approx(sum(c.reward_component_totals.values()))
                with pytest.raises(RuntimeError):c.step(np.zeros(4))
            assert controls<800
        # Unit clock fixture: horizon success and failure receive no tail cost.
        for stable in (None,3.5):
            c.reset(seed=0,options=opts)
            c.evaluator.completed_intervals=3995;c.evaluator.elapsed_s=3.995;c.evaluator.stable_since=stable
            if stable is None:
                # Metrics outside stable gates throughout the last control.
                c.plant.data.qvel[:3]=[.2,0,0]
            out=c.step(np.r_[c.plant.hover_force,0,0,0])
            assert out[2] and out[4]['result']['failure_tail']['terminal_reward']==0.
    finally:b.close();c.close()


def test_working_flight_observation_and_timeout_contract():
    from rm27_contact_rl.flight_working import WorkingFlightEnv, FeasibleCTBR
    raw = WorkingFlightEnv(ROOT/'configs/flight_working/attitude_baseline.json')
    env = FeasibleCTBR(raw)
    try:
        obs, _ = env.reset(options=dict(reference_index=1, initial_perturbation=np.zeros(12)))
        assert obs.shape == (12,)
        np.testing.assert_allclose(obs[:3], [1., 0., 0.], atol=1e-6)
        saved = raw.plant.data.act.copy()
        raw.plant.data.act[:] *= .5
        np.testing.assert_array_equal(raw._observation(), obs)
        raw.plant.data.act[:] = saved
        assert env.action(-np.ones(4))[0] > 0
        np.testing.assert_allclose(env.action(np.zeros(4)), [raw.plant.hover_force, 0, 0, 0])
        # A single actual interval with a terminal-history fixture verifies that
        # unsuccessful horizon scoring still bootstraps; not a full-flight claim.
        raw.evaluator.completed_intervals = 3999
        raw.evaluator.elapsed_s = 3.999
        raw.evaluator.stable_since = None
        _, reward, terminated, truncated, info = env.step(np.zeros(4))
        assert truncated and not terminated
        assert info['result']['failure_modes'] == ['NOT_STABLE_AT_HORIZON']
        assert info['reward_components']['event'] == 0 and reward > 0
    finally:
        env.close()
