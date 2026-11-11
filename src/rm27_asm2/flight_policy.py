"""Evaluate the frozen Ver2 Track actor on ASM2 without updating its weights."""

import io
from pathlib import Path

import numpy as np

from simpleflight_lab.stage1_native_eval import NativeActor
from .asset import ROOT, digest
from .flight_suite import flight_settings, rotation_matrix


def observations(suite, states, tick):
    times = tick*suite.dt + .05*np.arange(10)
    references = np.stack([suite.reference(t)[0] for t in times], axis=1)
    offsets = (references-states[:, None, :3]).reshape(len(states), 30)
    axes = rotation_matrix(states[:, 3:7]).transpose(0, 2, 1).reshape(len(states), 9)
    return np.column_stack((offsets, states[:, 7:10], axes, states[:, 10:13])).astype(np.float32)


class FrozenTrack:
    def __init__(self, profile, suite, weights, metadata):
        self.profile, self.suite = profile, suite
        self.layers = NativeActor.layers(weights, 45)
        self.metadata = {"name": "frozen_ver2_track", **metadata}

    def __call__(self, states, tick):
        # Historical actor order [p,q,r,T]; ASM2 scales collective around new mg.
        action = np.tanh(NativeActor.mean(observations(self.suite, states, tick), self.layers))
        collective = self.profile.hover_thrust_N * (1.0+.5*action[:, 3])
        return np.column_stack((collective, 2.*action[:, :3]))


def export_weights(path, profile, suite):
    """Load the pinned checkpoint in Torch; verify portable inference on real observations."""
    import torch
    from simpleflight_lab.ppo import Actor, UPSTREAM_REVISION
    from simpleflight_lab.rm27 import RM27_SIM_REVISION

    pin = flight_settings()["ver2_track_checkpoint"]
    checkpoint = ROOT / pin["path"]
    if digest(checkpoint) != pin["sha256"]:
        raise ValueError("Frozen Ver2 Track checkpoint changed")
    saved = torch.load(io.BytesIO(checkpoint.read_bytes()), map_location="cpu", weights_only=True)
    expected = {"format": 1, "task": "track", "airframe": "rm27", "actor_obs": 45,
                "upstream_revision": UPSTREAM_REVISION, "rm27_sim_revision": RM27_SIM_REVISION,
                "policy_dt": profile.policy_dt_s}
    if any(saved.get(k) != v for k, v in expected.items()):
        raise ValueError("Ver2 actor is incompatible with the expected observation/action/timing contract")
    actor = Actor(45).eval()
    actor.load_state_dict(saved["actor"])
    weights = {name: value.detach().numpy() for name, value in actor.state_dict().items()}
    obs = np.vstack([observations(suite, suite.initial, tick) for tick in (0, suite.steps//3, suite.steps-1)])
    with torch.inference_mode():
        expected_mean = actor(torch.from_numpy(obs)).numpy()
    observed_mean = NativeActor.mean(obs.copy(), NativeActor.layers(weights, 45))
    np.testing.assert_allclose(observed_mean, expected_mean, atol=2e-5, rtol=0)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        np.savez_compressed(stream, **weights, checkpoint_sha256=pin["sha256"],
                            torch_mean_max_abs_error=float(np.max(np.abs(observed_mean-expected_mean))))


def portable_controller_factory(path):
    """Load validated weights without a Torch or Isaac import in native MuJoCo."""
    with np.load(path, allow_pickle=False) as bundle:
        arrays = dict(bundle)
    checkpoint_sha = arrays.pop("checkpoint_sha256").item()
    if checkpoint_sha != flight_settings()["ver2_track_checkpoint"]["sha256"]:
        raise ValueError("Portable weights do not match the pinned Ver2 actor")
    metadata = {"checkpoint_sha256": checkpoint_sha, "weights_sha256": digest(path),
                "torch_mean_max_abs_error": arrays.pop("torch_mean_max_abs_error").item()}
    return lambda profile, suite: FrozenTrack(profile, suite, arrays, metadata)
