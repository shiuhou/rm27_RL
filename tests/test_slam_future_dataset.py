import csv
from pathlib import Path

import pytest

from slam_future_fixtures import make_dataset, LABEL
from rm27.perception.vision.localization.experiments.dataset import CanonicalDataset
from rm27.perception.vision.localization.experiments.calibration import validated_calibration
from rm27.perception.vision.localization.experiments.common import (
    ExperimentError, read_json, write_json, file_hash, _TEST_FIXTURE_TOKEN)


def test_reader_provenance_order_and_actual_subsets(tmp_path):
    path=make_dataset(tmp_path/'data')
    dataset=CanonicalDataset(path)
    frames=dataset.select('severe_blur')
    assert frames[0].source_frame_id==0 and frames[-1].source_frame_id==8
    assert frames[1].encoded_pts==format(1/180,'.17g')
    assert frames[0].timestamp_evidence['semantic']=='ENCODED_STREAM_PTS'
    assert frames[0].timestamp_evidence['clock_domain']=='UNKNOWN'
    assert dataset.provenance['source_sha256']==file_hash(path.parent/'source.mp4')
    with pytest.raises(ExperimentError,match='SUBSET_NOT_FOUND'):
        dataset.select('rotation-heavy')


@pytest.mark.parametrize('rate,step',[(180,1),(90,2),(60,3),(45,4),(30,6)])
def test_frame_rate_original_pts(tmp_path,rate,step):
    path=make_dataset(tmp_path/'data')
    dataset=CanonicalDataset(path)
    selected=dataset.select(input_hz=rate)
    assert [f.source_frame_id for f in selected]==list(range(0,18,step))
    assert selected[1].encoded_pts==dataset.frames[step].encoded_pts


def test_sparse_view_cannot_invent_180hz(tmp_path):
    dataset=CanonicalDataset(make_dataset(tmp_path/'data',step=6))
    assert len(dataset.select(input_hz=30))==3
    with pytest.raises(ExperimentError,match='RATE_UNAVAILABLE'):
        dataset.select(input_hz=180)


@pytest.mark.parametrize('failure',['source_missing','image_missing','source_hash','image_hash','image_geometry','frame_order','subset'])
def test_dataset_dependencies_fail_closed(tmp_path,failure):
    path=make_dataset(tmp_path/'data'); root=path.parent
    if failure=='source_missing':(root/'source.mp4').unlink()
    elif failure=='image_missing':(root/'images/0.png').unlink()
    elif failure=='source_hash':(root/'source.mp4').write_bytes(b'changed')
    elif failure=='image_hash':(root/'images/0.png').write_bytes(b'changed')
    elif failure=='image_geometry':
        import cv2,numpy as np
        cv2.imwrite(str(root/'images/0.png'),np.zeros((12,12),np.uint8))
        with (root/'frames.csv').open() as f: rows=list(csv.DictReader(f))
        rows[0]['image_sha256']=file_hash(root/'images/0.png')
        with (root/'frames.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    elif failure=='frame_order':
        with (root/'frames.csv').open() as f: rows=list(csv.DictReader(f))
        rows[1]['source_frame_index']='0'
        with (root/'frames.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    else:
        subset=read_json(root/'subsets.json');subset['severe_blur']['source_frame_indices']=[999]
        write_json(root/'subsets.json',subset)
    with pytest.raises(ExperimentError):CanonicalDataset(path)


def test_calibration_fixture_is_only_allowed_privately(tmp_path):
    dataset=CanonicalDataset(make_dataset(tmp_path/'data'))
    with pytest.raises(ExperimentError,match='TEST_FIXTURE_FORBIDDEN'):
        validated_calibration(dataset)
    c,p=validated_calibration(dataset,_fixture_token=_TEST_FIXTURE_TOKEN)
    assert c['purpose']==LABEL and p['sha256']==file_hash(Path(p['path']))


@pytest.mark.parametrize('failure',['required','unverified','missing','hash','geometry','id','configuration','report','report_hash','nan','bool'])
def test_calibration_gate_failures(tmp_path,failure):
    path=make_dataset(tmp_path/'data');root=path.parent
    m=read_json(path);c=read_json(root/'calibration.json')
    if failure=='required':m['status']='CALIBRATION_REQUIRED'
    elif failure=='unverified':c['status']='UNVERIFIED_CALIBRATION'
    elif failure=='missing':m['calibration']['file']='missing.json'
    elif failure=='geometry':c['image_width']=128
    elif failure=='id':c['calibration_id']='different'
    elif failure=='configuration':c['lens_id']='different'
    elif failure=='report':c['validation']['status']='FAILED'
    elif failure=='report_hash':c['validation']['report_sha256']='0'*64
    elif failure=='bool':c['fx']=True
    elif failure=='nan':
        (root/'calibration.json').write_text('{"fx": NaN}')
    if failure!='nan':write_json(root/'calibration.json',c)
    m['calibration']['sha256']='0'*64 if failure=='hash' else file_hash(root/'calibration.json')
    write_json(path,m)
    with pytest.raises((ExperimentError,ValueError)):
        validated_calibration(CanonicalDataset(path),_fixture_token=_TEST_FIXTURE_TOKEN)
