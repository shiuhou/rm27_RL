"""Bounded development ablation of rate feedforward at the contact reset.

No held-out seeds or trajectory selection are used by this study. In particular,
lower peak angular rate alone is not declared a successful recovery.
"""
import copy
from pathlib import Path

from .campaign import aggregate
from .evidence import snapshot, write_json
from .experiment import run_episode


def rate_ablation(cfg, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    variants = {"normal_feedforward": {}, "blank_50ms": {"recovery_rate_blank_s": .05},
                "blank_200ms": {"recovery_rate_blank_s": .2},
                "no_recovery_feedforward": {"recovery_rate_feedforward": False}}
    stats = {}
    for name, override in variants.items():
        c = copy.deepcopy(cfg)
        c["controller"].update(override)
        rows = [run_episode(c, seed=seed, output=output/name/f"seed-{seed}")
                for seed in cfg["search"]["training_seeds"]]
        stats[name] = aggregate(rows)
    result = dict(passed=True, passed_scope="development ablation completed; see individual outcomes",
                  variants=variants, results=stats, training_seeds=cfg["search"]["training_seeds"],
                  used_for_primary_selection=False, hardware_validated=False)
    write_json(output/"summary.json", result)
    return result
