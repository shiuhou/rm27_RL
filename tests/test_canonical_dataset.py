import csv
import json
import sys

import pytest
from rm27.perception.vision.localization import prepare_vsl2_dataset as dataset


def fixture(tmp_path):
    source = tmp_path/'upstream'
    (source/'quality').mkdir(parents=True)
    (source/'frames').mkdir()
    video = tmp_path/'raw.mp4'
    video.write_bytes(b'raw test evidence')
    image = source/'frames/frame.jpg'
    image.write_bytes(b'derived test image')
    metadata = dict(source=dict(path=str(video), sha256=dataset.sha256(video), git_commit='test'),
                    video_stream=dict(width=1344, height=760, nominal_fps=180),
                    decoder=dict(backend='TEST'), sampling=dict(derived_images='test'))
    (source/'video_stream.json').write_text(json.dumps(metadata))
    (source/'intervals.json').write_text('[]')
    row = dict(frame_index=0, pts_time_s=0, image_file='frames/frame.jpg')
    row.update({k: 0 for k in ('laplacian_variance', 'feature_count', 'empty_feature_cells',
               'underexposed_fraction', 'overexposed_fraction', 'median_flow_px', 'rotation_proxy_deg')})
    with (source/'quality/frame_quality.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    return source, video, image


@pytest.mark.parametrize('broken', [None, 'video', 'image'])
def test_view_checks_dependencies_without_copying(tmp_path, monkeypatch, broken):
    source, video, image = fixture(tmp_path)
    if broken == 'video':
        video.write_bytes(b'changed source')
    if broken == 'image':
        image.unlink()
    out = tmp_path/'view'
    monkeypatch.setattr(sys, 'argv', ['prepare_vsl2_dataset', '--vsl1', str(source), '--output', str(out)])
    if broken:
        with pytest.raises(ValueError):
            dataset.main()
        assert not (out/'dataset_manifest.json').exists()
    else:
        dataset.main()
        manifest = json.loads((out/'dataset_manifest.json').read_text())
        assert manifest['source']['image_path_base'] == 'source.artifact_directory'
        assert manifest['source']['upstream_metadata_sha256']['video_stream.json']
        assert json.loads((out/'image_hashes.json').read_text())['frames/frame.jpg'] == dataset.sha256(image)
        assert not (out/'frames').exists()
