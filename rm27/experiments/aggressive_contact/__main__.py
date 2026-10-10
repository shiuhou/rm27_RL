import argparse
import json
from pathlib import Path

from .experiment import run_episode
from .model import DEFAULT_CONFIG, load_config
from .evidence import save_failure


def main():
    parser = argparse.ArgumentParser(description="RM27 aggressive contact/recovery research (simulation only)")
    parser.add_argument("command", choices=["episode", "campaign", "recovery-map", "validate", "rate-ablation", "plot", "replay",
                                           "arena", "view", "render", "arena-audit", "robust-campaign", "causal-study", "validate-v2",
                                           "maneuver", "maneuver-campaign", "maneuver-audit", "maneuver-pose-study", "maneuver-validate", "maneuver-plot", "render-maneuver", "teacher-campaign", "recoverability-study", "teacher-export", "teacher-return", "teacher-plot", "teacher-interfaces", "teacher-return-campaign", "recoverability-audit", "teacher-validate"])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--pose-input", type=Path)
    parser.add_argument("--parameters", type=Path)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--candidates", type=int)
    parser.add_argument("--trials", type=int)
    parser.add_argument("--scene", choices=["full", "isolated"], default="full")
    parser.add_argument("--view", action="store_true")
    parser.add_argument("--speed", type=float, default=1.)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument('--camera',choices=['follow','overview','fpv'],default='follow',help='Viewer camera; fpv also supported by render/render-maneuver')
    parser.add_argument('--fovy',type=float,help='First-person vertical field of view in degrees')
    from .fpv import DEFAULT_MOUNT
    parser.add_argument('--camera-config',type=Path,default=DEFAULT_MOUNT,help='Display-only first-person camera mount')
    parser.add_argument("--scale", type=float, default=1.)
    args = parser.parse_args()
    if args.command in ("view", "render", "arena-audit", "replay", "plot", "validate", "validate-v2", "maneuver-audit", "maneuver-pose-study", "maneuver-plot", "render-maneuver") and args.input is None:
        parser.error(f"{args.command} requires --input DIRECTORY")
    if args.command == "view":
        if args.input is None:
            parser.error("view requires --input EPISODE_DIRECTORY")
        from .viewing import view_episode
        view_episode(args.input, args.speed, args.loop, args.camera, args.fovy, args.camera_config)
        return
    if args.output is None:
        parser.error("--output is required for this command")
    if args.output.exists():
        parser.error(f"Output directory already exists: {args.output}")
    try:
        cfg = load_config(args.config)
        if args.command in ('teacher-interfaces','teacher-return-campaign','recoverability-audit'):
            if args.input is None:
                parser.error(f'{args.command} requires --input DIRECTORY')
            if args.command == 'teacher-interfaces':
                from .teacher import interface_campaign
                result=interface_campaign(cfg,args.input,args.output)
            elif args.command == 'teacher-return-campaign':
                from .teacher_return import campaign
                result=campaign(cfg,args.input,args.output)
            else:
                from .recoverability import failure_audit
                result=failure_audit(args.input,args.output)
        elif args.command == 'teacher-validate':
            from .teacher_validation import run
            if args.input is None:
                parser.error('teacher-validate requires --input CAMPAIGN')
            result=run(cfg,args.input,args.output)
        elif args.command == 'teacher-campaign':
            from .teacher import run
            result = run(cfg, args.output)
        elif args.command == 'recoverability-study':
            from .recoverability import study
            result = study(args.output)
        elif args.command == 'teacher-plot':
            from .teacher_tools import plot_campaign
            if args.input is None:
                parser.error('teacher-plot requires --input CAMPAIGN')
            result = plot_campaign(args.input,args.output)
        elif args.command == 'teacher-export':
            from .teacher_tools import export_campaign
            if args.input is None:
                parser.error('teacher-export requires --input CAMPAIGN')
            result = export_campaign(args.input,args.output)
        elif args.command == 'teacher-return':
            from .teacher_return import mission
            if args.parameters is None:
                parser.error('teacher-return requires --parameters DESIGN.json')
            result = mission(cfg,json.loads(args.parameters.read_text()),args.output,args.seed,args.scale)
        elif args.command == 'maneuver':
            from .maneuver import episode, settings
            design = json.loads(args.parameters.read_text()) if args.parameters else settings()['initial_design']
            result = episode(cfg, design, args.output, args.seed, args.scale, view=args.view, speed=args.speed, finish_on_recovery=True)
        elif args.command == 'maneuver-campaign':
            from .maneuver import campaign
            result = campaign(cfg, args.output, args.candidates, args.trials)
        elif args.command == 'maneuver-pose-study':
            from .maneuver import early_pose_study
            result = early_pose_study(cfg, args.input, args.output, args.trials)
        elif args.command == 'maneuver-validate':
            from .maneuver import validate
            result = validate(cfg, args.output)
        elif args.command == 'maneuver-audit':
            from .arena_audit import audit_routes
            result = audit_routes(args.input, args.output, approach=True)
        elif args.command == "arena":
            from .arena import mission
            from .robust import configured
            # Use the explicit v2 observer/adaptation for the arena default.
            cfg = configured(cfg, "adaptive_nominal")
            p = json.loads(args.parameters.read_text()) if args.parameters else cfg["nominal_skill"]
            result = mission(cfg, p, args.output, args.scene, args.view, args.speed, args.seed, args.scale)
        elif args.command == "robust-campaign":
            from .robust import robust_campaign
            result = robust_campaign(cfg, args.output, args.candidates, args.trials)
        elif args.command == "causal-study":
            from .robust import causal_study
            result = causal_study(cfg, args.output, args.trials or 32)
        elif args.command == "validate-v2":
            from .robust import validate_v2
            result = validate_v2(cfg, args.input, args.output)
        elif args.command == 'maneuver-plot':
            from .maneuver_plot import plot
            result = plot(args.input, args.output, args.pose_input)
        elif args.command in ("render", 'render-maneuver'):
            if args.camera=='fpv':
                from .fpv import render_fpv
                result=render_fpv(args.input,args.output,fovy=args.fovy,camera_config=args.camera_config,speed=args.speed)
            else:
                from .viewing import render_episode
                result = render_episode(args.input, args.output, whole=args.command == 'render-maneuver')
        elif args.command == "arena-audit":
            from .arena_audit import audit_routes
            result = audit_routes(args.input, args.output)
        elif args.command == "episode":
            p = json.loads(args.parameters.read_text()) if args.parameters else None
            result = run_episode(cfg, p, args.seed, args.scale, output=args.output, record_sources=True)
        elif args.command == "campaign":
            from .campaign import campaign
            result = campaign(cfg, args.output, args.candidates or 32, args.trials or 32)
        elif args.command == "recovery-map":
            from .recovery import recovery_map
            result = recovery_map(cfg, args.output)
        elif args.command == "validate":
            from .studies import validate
            result = validate(cfg, args.input, args.output)
        elif args.command == "rate-ablation":
            from .ablation import rate_ablation
            result = rate_ablation(cfg, args.output)
        else:
            from .visualize import plot, replay
            result = (plot if args.command == "plot" else replay)(args.input, args.output)
    except Exception as exc:
        save_failure(args.output, exc)
        if args.config.exists():
            (args.output/"config-input.json").write_bytes(args.config.read_bytes())
        raise
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
