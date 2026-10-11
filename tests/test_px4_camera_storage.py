import json

import cv2
import numpy as np
import pytest

from tools.px4 import camera_storage


@pytest.fixture
def camera(tmp_path):
    if not (camera_storage.ROOT/'artifacts/px4-lossless-video-runtime-01/summary.json').is_file():
        pytest.skip('Optional verified lossless video runtime is absent')
    root=tmp_path/'return_camera'
    images=root/'images'
    images.mkdir(parents=True)
    frame=np.random.default_rng(801).integers(0,256,(256,256),dtype=np.uint8)
    originals={}
    for index in range(24):
        name=f'{index:06d}-{index*50000000}.png'
        assert cv2.imwrite(str(images/name),frame)
        originals[name]=(images/name).read_bytes()
    return root,originals


def test_return_camera_archive_restores_original_png_bytes(camera):
    root,originals=camera
    archived=camera_storage.archive(root)
    assert archived['passed'] and archived['storage_changed']
    assert not list((root/'images').glob('*.png'))
    record=json.loads((root/'images.archive.json').read_text())
    assert [r['filename'] for r in record['records']]==list(originals)
    assert camera_storage.restore(root)['restored_images']==len(originals)
    assert {p.name:p.read_bytes() for p in (root/'images').glob('*.png')}==originals


def test_camera_roundtrip_failure_keeps_every_original(camera,monkeypatch):
    root,originals=camera
    def reject(*_):
        raise ValueError('Decoded PNG differs from the original')
    monkeypatch.setattr(camera_storage,'decode',reject)
    with pytest.raises(ValueError,match='Decoded PNG differs'):
        camera_storage.archive(root)
    assert {p.name:p.read_bytes() for p in (root/'images').glob('*.png')}==originals
    assert not (root/'images.lossless.mkv').exists()
    assert not (root/'images.archive.json').exists()
