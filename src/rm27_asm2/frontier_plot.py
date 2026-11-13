"""Render matched Phase R reset outcomes from the saved native evaluation."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


CONTROLLERS = {
    "phase4": "Phase 4 PPO",
    "fixed_a": "Fixed curriculum A",
    "deterministic": "Action-limited deterministic",
}
PHASE_ORDER = (
    "handover", "early", "rotation", "transition_pre", "transition_post",
    "braking_mid", "braking_late", "near_contact",
)
MARKERS = ("D", "X", "s", "p", "o", "v", "^", "P", "<", ">", "h", "8")
OUTCOMES = {
    "None qualified": "#d3dce4",
    "Some qualified": "#e49a2c",
    "All qualified": "#147c85",
}


def _anchors(rows):
    metadata = {}
    trials = {}
    for row in rows:
        anchor = row["case"], row["frame"]
        lead_s = float(row["lead_s"])
        phase = row["phase"]
        controller = row["controller"]
        variant = row["variant"]
        if not math.isfinite(lead_s) or lead_s <= 0:
            raise ValueError(f"Expected a positive witness lead for {anchor}, got {lead_s}")
        if controller not in CONTROLLERS or not isinstance(row["qualified"], bool):
            raise ValueError(f"Invalid controller or qualification for {anchor}")
        if metadata.setdefault(anchor, (lead_s, phase)) != (lead_s, phase):
            raise ValueError(f"Conflicting lead or phase for {anchor}")
        group = trials.setdefault((anchor, controller), {})
        if variant in group:
            raise ValueError(f"Duplicate variant {variant} for {anchor} and {controller}")
        group[variant] = row["qualified"]
    if not metadata:
        raise ValueError("No frontier rollouts to plot")
    for anchor in metadata:
        variants = [set(trials.get((anchor, controller), {})) for controller in CONTROLLERS]
        if not variants[0] or any(group != variants[0] for group in variants[1:]):
            raise ValueError(f"Controller variants differ for {anchor}")
    return metadata, trials


def plot_frontier(directory: Path) -> Path:
    """Plot one point per matched reset anchor, aggregated across its variants."""
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FuncFormatter, LogLocator

    directory = Path(directory)
    rows = json.loads((directory / "results.json").read_text())["rows"]
    metadata, trials = _anchors(rows)
    witnesses = sorted({case for case, _ in metadata})
    witness_y = {case: i for i, case in enumerate(witnesses)}
    observed = {phase for _, phase in metadata.values()}
    phases = [phase for phase in PHASE_ORDER if phase in observed]
    phases.extend(sorted(observed - set(phases)))
    if len(phases) > len(MARKERS):
        raise ValueError("Too many reset phases for distinct plot markers")
    markers = dict(zip(phases, MARKERS))
    leads = [lead for lead, _ in metadata.values()]
    height = max(5.0, 3.0 + .34 * len(witnesses))
    fig, axes = plt.subplots(1, len(CONTROLLERS), figsize=(15, height), sharey=True, sharex=True)
    fig.subplots_adjust(left=.13, right=.985, bottom=.24, top=.82, wspace=.085)
    fig.suptitle("ASM2 policy success frontier", y=.965, fontsize=19, fontweight="bold")
    fig.text(.5, .9, f"{len(metadata)} matched reset anchors across {len(witnesses)} development witnesses",
             ha="center", fontsize=11, color="#3b4654")

    for ax, (controller, title) in zip(axes, CONTROLLERS.items(), strict=True):
        ax.set_title(title, fontsize=12, pad=11, fontweight="bold")
        ax.set_xscale("log")
        ax.set_xlim(max(leads) * 1.18, min(leads) / 1.18)
        ax.xaxis.set_major_locator(LogLocator(base=10, subs=(1., 2., 5.), numticks=12))
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{x:g}"))
        ax.set_ylim(len(witnesses) - .5, -.5)
        ax.set_yticks(range(len(witnesses)), witnesses)
        ax.tick_params(labelsize=9)
        ax.set_axisbelow(True)
        ax.grid(axis="x", color="#e5e9ef", linewidth=.75)
        for y in range(len(witnesses)):
            if y % 2 == 0:
                ax.axhspan(y - .5, y + .5, color="#f5f7fa", zorder=0)
        for spine in ax.spines.values():
            spine.set_color("#c1cbd6")
        for anchor, (lead_s, phase) in metadata.items():
            values = trials[anchor, controller].values()
            outcome = "All qualified" if all(values) else "Some qualified" if any(values) else "None qualified"
            # The two transition frames are one control tick apart. Offset their
            # markers vertically to show both without shifting their measured lead.
            phase_offset = {"transition_pre": -.17, "transition_post": .17}.get(phase, 0.)
            ax.scatter(lead_s, witness_y[anchor[0]] + phase_offset, marker=markers[phase], s=54,
                       facecolor=OUTCOMES[outcome], edgecolor="#314755", linewidth=.65,
                       zorder={"None qualified": 2, "Some qualified": 3, "All qualified": 4}[outcome])

    axes[0].set_ylabel("Development witness", labelpad=9)
    fig.text(.5, .156, "Time remaining to source-witness contact at reset (s, log scale)",
             ha="center", fontsize=10)
    shape_handles = [Line2D([], [], marker=markers[phase], linestyle="None", markerfacecolor="#ffffff",
                            markeredgecolor="#314755", label=phase.replace("_", " ")) for phase in phases]
    fill_handles = [Line2D([], [], marker="o", linestyle="None", markerfacecolor=color,
                           markeredgecolor="#314755", label=label) for label, color in OUTCOMES.items()]
    fig.legend(handles=shape_handles, loc="lower center", bbox_to_anchor=(.5, .075),
               ncol=len(shape_handles), frameon=False, title="Reset phase", fontsize=9)
    fig.legend(handles=fill_handles, loc="lower center", bbox_to_anchor=(.5, .005),
               ncol=3, frameon=False, title="Contact across variants", fontsize=9)
    output = directory / "frontier.png"
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="Phase R run directory containing results.json")
    args = parser.parse_args()
    print(plot_frontier(args.directory))


if __name__ == "__main__":
    main()
