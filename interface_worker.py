"""Synthetic JSONL interface endpoint. No hardware, backend estimator or training is started."""
import contextlib
import importlib.metadata
import json
from pathlib import Path
import sys

def identity():
    modules = {n: str(Path(m.__file__).resolve()) for n, m in sys.modules.items()
               if getattr(m, "__file__", None) and (n == "interface_worker" or n.startswith(("rm27.", "rm27_contact_")))}
    versions = {}
    for name in ("numpy", "scipy", "mujoco", "pymavlink", "torch", "gymnasium", "stable-baselines3"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            pass
    return {"source": SOURCE, "worker": str(Path(__file__).resolve()), "python": sys.executable,
            "python_version": sys.version, "modules": modules, "versions": versions,
            "qualification": "SYNTHETIC_INTERFACE_ONLY"}

SOURCE = "rl"

import numpy as np
import torch
from stable_baselines3.common.policies import ActorCriticPolicy
from rm27_contact_physics.model import VehicleEstimate
from rm27_contact_rl.env import ContactRecoveryEnv, contact_observation
from rm27_contact_rl.ppo import NormalizedCTBR
wrapped = None
actor = None

def handle(kind, p):
    global wrapped, actor
    if kind == "hello": return identity()
    if kind == "infer":
        if p.get("observation_schema") != "contact_world_v1": raise ValueError("wrong observation schema")
        if wrapped is None:
            torch.set_num_threads(1)
            torch.manual_seed(p["seed"])
            wrapped=NormalizedCTBR(ContactRecoveryEnv())
            actor=ActorCriticPolicy(wrapped.observation_space,wrapped.action_space,lr_schedule=lambda _:0.)
            actor.set_training_mode(False)
        state=VehicleEstimate(**{k:np.array(v,float) if isinstance(v,list) else v for k,v in p["state"].items()})
        obs=contact_observation(state,np.array(p["target_position"]),np.array(p["target_normal"]),
                                np.array(p["previous_action"]),p["remaining_s"])
        if obs.shape != (26,) or not np.isfinite(obs).all(): raise ValueError("invalid observation")
        with torch.no_grad():
            normalized,_=actor.predict(obs,deterministic=True)
        action=wrapped.action(normalized)
        return {"observation":obs.tolist(),"normalized":normalized.tolist(),"action":action.tolist(),
                "maximum_collective_N":float(wrapped.high[0]),"policy":"UNTRAINED_FIXED_SEED",
                "learning_steps":0,"observation_schema":"contact_world_v1"}
    raise ValueError("unsupported kind")

def main():
    previous = -1
    for line in sys.stdin:
        request = None
        try:
            request = json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError("nonfinite JSON")))
            if (type(request.get("version")) is not int or request.get("version") != 1 or request.get("source") != "coordinator"
                    or type(request.get("seq")) is not int or request["seq"] <= previous
                    or request.get("input_kind") != "synthetic"):
                raise ValueError("invalid envelope/source/sequence/input_kind")
            previous = request["seq"]
            with contextlib.redirect_stdout(sys.stderr):
                result = handle(request["kind"], request["payload"])
            reply = {"version": 1, "source": SOURCE, "seq": request["seq"],
                     "input_kind": "synthetic", "kind": request["kind"], "ok": True, "payload": result}
        except Exception as exc:
            reply = {"version": 1, "source": SOURCE, "seq": request.get("seq") if isinstance(request, dict) else None,
                     "input_kind": "synthetic", "kind": request.get("kind") if isinstance(request, dict) else None,
                     "ok": False, "error": str(exc), "error_type": type(exc).__name__}
        print(json.dumps(reply, allow_nan=False), flush=True)

if __name__ == "__main__":
    main()
