import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from rm27.perception.vision.localization import analyze_video as analyzer


def test_acceptance_is_not_eof():
    status = analyzer.acceptance_status(10, 3, [0, .1, .2], 0, [], True)
    assert status['execution_completed'] and status['premature_decode_failure']
    assert not status['passed'] and not status['integrity_checks_passed']
    assert not analyzer.acceptance_status(0, 3, [0, .1, .2], 0, [], True)['passed']
    assert not analyzer.acceptance_status(3, 3, [0, 0, .2], 0, [], True)['passed']
    assert not analyzer.acceptance_status(3, 3, [0, .1, .2], 1, [], True)['passed']
    assert not analyzer.acceptance_status(3, 3, [0, .1, .2], 0, [], False)['passed']
    assert analyzer.acceptance_status(3, 3, [5, 5.1, 5.2], 0, [], True)['passed']


class FakeCapture:
    def __init__(self, count=3):
        self.index, self.count = 0, count
        self.frames = [np.random.default_rng(i).integers(0, 255, (48, 64, 3), dtype=np.uint8) for i in range(3)]
    def isOpened(self):
        return True
    def getBackendName(self):
        assert self.index <= 3
        return 'TEST_BACKEND'
    def get(self, prop):
        return {cv2.CAP_PROP_FPS: 10, cv2.CAP_PROP_FRAME_WIDTH: 64, cv2.CAP_PROP_FRAME_HEIGHT: 48,
                cv2.CAP_PROP_FRAME_COUNT: self.count, cv2.CAP_PROP_FOURCC: 0,
                cv2.CAP_PROP_POS_MSEC: 5000 + max(0, self.index-1)*100}[prop]
    def read(self):
        if self.index == 3:
            return False, None
        image = self.frames[self.index]
        self.index += 1
        return True, image
    def release(self):
        pass


@pytest.mark.parametrize('count,write_ok,passed', [(3, True, True), (5, True, False), (3, False, False)])
def test_main_status_writes_and_provenance(tmp_path, monkeypatch, count, write_ok, passed):
    source = tmp_path / 'test.mp4'
    source.write_bytes(b'synthetic fixture, not actual video')
    out = tmp_path / 'analysis'
    def capture(_):
        assert json.loads((out/'provenance.json').read_text())['source_sha256_before'] == analyzer.sha256(source)
        return FakeCapture(count)
    monkeypatch.setattr(analyzer.cv2, 'VideoCapture', capture)
    if not write_ok:
        monkeypatch.setattr(analyzer.cv2, 'imwrite', lambda *a: False)
    result = analyzer.main([str(source), '--out', str(out), '--sample-step', '1'])
    assert result == (0 if passed else 1)
    report = json.loads((out/'analysis_complete.json').read_text())
    assert report['passed'] is passed
    assert bool(report['write_failures']) is not write_ok
    provenance = json.loads((out/'provenance.json').read_text())
    for key in ('source_sha256_before', 'source_sha256_after', 'analyzer_file_sha256', 'git_commit',
                'interpreter', 'opencv_version', 'parameters', 'analysis_command'):
        assert key in provenance
    assert str(source) in provenance['analysis_command'] and '--sample-step 1' in provenance['analysis_command']
    assert provenance['backend'] == 'TEST_BACKEND'
    stream = report['metadata']['video_stream']
    assert stream['first_pts_s'] == 5 and stream['last_pts_s'] == 5.2
    assert stream['pts_span_s'] == pytest.approx(.2)
    assert stream['container_duration_s'] is None
