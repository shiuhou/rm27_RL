"""Generate a printable chessboard SVG; printed dimensions must be measured."""
import argparse
import math
from pathlib import Path


def chessboard_svg(corners_x=9, corners_y=6, square_mm=25.0):
    if type(corners_x) is not int or type(corners_y) is not int or min(corners_x, corners_y) < 3:
        raise ValueError("inner corner counts must be integers >= 3")
    if isinstance(square_mm, bool) or not isinstance(square_mm, (int, float)) or not math.isfinite(square_mm) or square_mm <= 0:
        raise ValueError("nominal square_mm must be finite and positive")
    margin = square_mm
    width, height = ((corners_x+3)*square_mm, (corners_y+3)*square_mm)
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}mm" height="{height}mm" viewBox="0 0 {width} {height}">',
             f'<rect width="{width}" height="{height}" fill="white"/>']
    for y in range(corners_y+1):
        for x in range(corners_x+1):
            if (x+y) % 2 == 0:
                lines.append(f'<rect x="{margin+x*square_mm}" y="{margin+y*square_mm}" width="{square_mm}" height="{square_mm}" fill="black"/>')
    return '\n'.join(lines + ['</svg>\n'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--corners-x', type=int, default=9)
    parser.add_argument('--corners-y', type=int, default=6)
    parser.add_argument('--square-mm', type=float, default=25.0, help='nominal printing size, not a measurement')
    args = parser.parse_args()
    svg = chessboard_svg(args.corners_x, args.corners_y, args.square_mm)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x') as stream:
        stream.write(svg)
    print(f'{args.out}: print at 100%, measure both axes before entering square_size_m')


if __name__ == '__main__':
    main()
