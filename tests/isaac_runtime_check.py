"""Run inside Isaac Python: validate the packages selected after Kit startup."""

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import traceback


def check_runtime():
    import numpy as np
    import torch
    import torchvision
    import warp as wp
    from isaaclab.utils.warp import convert_to_warp_mesh, raycast_mesh

    assert wp.__version__ == '1.8.1', wp.__version__
    assert '/omni.warp.core-1.8.1/' in wp.__file__, wp.__file__
    assert torch.__version__ == '2.5.1+cu118', torch.__version__
    assert torchvision.__version__ == '0.20.1+cu118', torchvision.__version__
    assert '/omni.isaac.ml_archive-' in torchvision.__file__, torchvision.__file__
    assert torch.cuda.is_available()
    values = torch.arange(1024, device='cuda', dtype=torch.float32)
    torch.testing.assert_close((values * 3 + 2).cpu(), torch.arange(1024) * 3.0 + 2)

    # Compile and execute the real Isaac Lab Warp raycast kernel, including
    # shared Torch/Warp GPU buffers and both a physical hit and a missed ray.
    mesh = convert_to_warp_mesh(
        np.array([[-1., -1., 0.], [1., -1., 0.], [1., 1., 0.], [-1., 1., 0.]]),
        np.array([[0, 1, 2], [0, 2, 3]]), device='cuda:0')
    starts = torch.tensor([[[.1, .2, 1.], [2., 2., 1.]]], device='cuda')
    directions = torch.tensor([[[0., 0., -1.], [0., 0., -1.]]], device='cuda')
    hits, distance, normal, faces = raycast_mesh(
        starts, directions, mesh, return_distance=True, return_normal=True, return_face_id=True)
    wp.synchronize_device('cuda:0')
    torch.testing.assert_close(hits[0, 0], starts.new_tensor([.1, .2, 0.]))
    torch.testing.assert_close(distance[0, 0], starts.new_tensor(1.))
    torch.testing.assert_close(normal[0, 0], starts.new_tensor([0., 0., 1.]))
    assert int(faces[0, 0]) in (0, 1)
    assert torch.isinf(hits[0, 1]).all() and torch.isinf(distance[0, 1])
    assert int(faces[0, 1]) == -1

    image = torch.arange(192, dtype=torch.uint8).reshape(3, 8, 8)
    png = torchvision.io.encode_png(image)
    with tempfile.TemporaryDirectory() as directory:
        path = str(Path(directory) / 'roundtrip.png')
        torchvision.io.write_file(path, png)
        stored = torchvision.io.read_file(path)
        assert torch.equal(stored, png)
        assert torch.equal(torchvision.io.decode_png(stored), image)
        assert torch.equal(torchvision.io.read_image(path), image)
    jpeg_source = torch.full((3, 16, 16), 64, dtype=torch.uint8)
    jpeg = torchvision.io.encode_jpeg(jpeg_source, quality=95)
    jpeg_gpu = torchvision.io.decode_jpeg(jpeg, device='cuda:0')
    assert jpeg_gpu.is_cuda and jpeg_gpu.shape == jpeg_source.shape
    assert int((jpeg_gpu.cpu().short() - jpeg_source.short()).abs().max()) <= 3
    boxes = torch.tensor([[0., 0., 1., 1.], [0., 0., 1., 1.], [2., 2., 3., 3.]], device='cuda')
    scores = torch.tensor([.9, .8, .7], device='cuda')
    assert torchvision.ops.nms(boxes, scores, .5).tolist() == [0, 2]
    return {
        'passed': True, 'warp': wp.__version__, 'warp_path': wp.__file__,
        'torch': torch.__version__, 'torchvision': torchvision.__version__,
        'torchvision_path': torchvision.__file__, 'gpu': torch.cuda.get_device_name(0),
        'checks': ['torch_cuda_arithmetic', 'isaaclab_warp_raycast_hit_and_miss',
                   'png_file_read_write_and_lossless_roundtrip', 'cuda_jpeg_decode', 'cuda_nms'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose an unused output file')
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True)
    try:
        result = check_runtime()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print('ISAAC_RUNTIME_CHECK_PASS', json.dumps(result), flush=True)
        code = 0
    except Exception:
        traceback.print_exc()
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    # Follow the existing bounded diagnostics: Kit shutdown can stall.
    os._exit(code)


if __name__ == '__main__':
    main()
