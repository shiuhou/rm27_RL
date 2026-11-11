"""Run the frozen Phase 3 regression with deterministic feedback capped to the policy map."""
from argparse import ArgumentParser
from pathlib import Path
from unittest.mock import patch

import fresh_evaluation
from rm27_asm2 import terminal_native_eval

load_reference = terminal_native_eval.load_deterministic


def load_action_limited_deterministic(directory, manifest, populations, profile, spec):
    references, source = load_reference(directory, manifest, populations, profile, spec)
    bounded = {case: (fresh_evaluation.ClippedReference(ref, profile.hover_thrust_N), gate)
               for case, (ref, gate) in references.items()}
    return bounded, dict(
        source, action_map=dict(rate_abs_rad_s=[2., 2., 2.], thrust_hover_range=[.5, 1.5]),
        action_map_wrapper_sha256=terminal_native_eval.digest(Path(__file__)),
        action_map_implementation_sha256=terminal_native_eval.digest(fresh_evaluation.__file__),
    )


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--actor", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-dir", type=Path)
    parser.add_argument("--workers", type=int, default=8, choices=range(1, 17))
    args = parser.parse_args()
    if args.output.exists() or (args.replay_dir and args.replay_dir.exists()):
        parser.error("Choose new output and replay paths")
    args.suite = "heldout"
    args.experiment = fresh_evaluation.CONFIG
    args.controls = True
    args.deterministic = True
    with patch.object(terminal_native_eval, "load_deterministic", load_action_limited_deterministic):
        terminal_native_eval.evaluation(args)


if __name__ == "__main__":
    main()
