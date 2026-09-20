"""Independent dart experiment. Run: python -m research.Darts --help."""

import argparse
import json
from pathlib import Path
from .experiment import build,DEFAULT_CONFIG


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    p=commands.add_parser('build',help='Rebuild current dock field, import dart CAD and validate a gravity-only path')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    p.add_argument('--cad-assets',type=Path,help='Reuse an earlier Darts CAD audit directory with matching STEP hash')
    p=commands.add_parser('view',help='Live MuJoCo playback; no SITL required')
    p.add_argument('run',type=Path)
    p.add_argument('--speed',type=float,default=.25,help='Playback multiplier; default 0.25x')
    p.add_argument('--camera',choices=['overview','follow','target','launch','side'],default='overview')
    p=commands.add_parser('render',help='Save scene images, trajectory plot and optional video')
    p.add_argument('run',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--video',action='store_true')
    p.add_argument('--speed',type=float,default=.25)
    p.add_argument('--camera',choices=['overview','follow','target','launch','side'],default='follow')
    args=parser.parse_args()
    if args.command=='build':
        r=build(args.output,args.config,args.cad_assets)
        print(json.dumps({k:r[k] for k in ['passed','launch_m','target_m','flight_time_s','launch_speed_m_s',
                                         'apex_height_m','impact','centerline_checks']},indent=2))
        return 0 if r['passed'] else 1
    if args.command=='view':
        from .view import view
        view(args.run,args.speed,args.camera)
    else:
        from .render import render
        print(render(args.run,args.output,args.video,args.speed,args.camera))
    return 0


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (ValueError,FileExistsError,FileNotFoundError) as exc:
        raise SystemExit(str(exc)) from exc
