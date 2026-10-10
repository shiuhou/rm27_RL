"""Run and inspect the independent dart interception scenario."""
import argparse
from pathlib import Path

from .experiment import simulate
from .view import render, view


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run")
    run.add_argument("--output", required=True)
    run.add_argument("--config", default=None)
    run.add_argument("--passive", action="store_true")
    show = sub.add_parser("view")
    show.add_argument("run")
    show.add_argument("--speed", type=float, default=1.)
    show.add_argument("--camera", choices=["free", "auto", "overview", "target", "follow"], default="free")
    video = sub.add_parser("render")
    video.add_argument("run")
    video.add_argument("--output", required=True)
    video.add_argument("--video", action="store_true")
    args = parser.parse_args()
    if args.command == "run":
        from .experiment import DEFAULT_CONFIG
        print(simulate(args.output, args.config or DEFAULT_CONFIG, not args.passive))
    elif args.command == "view":
        view(Path(args.run), args.speed, args.camera)
    else:
        render(Path(args.run), Path(args.output), args.video)


if __name__ == "__main__":
    main()
