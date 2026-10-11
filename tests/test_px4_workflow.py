"""The supported workflow cannot claim success from a flight exit alone."""
import json

from rm27.autopilot.px4 import simulation


def test_nominal_flight_without_actual_ev_fusion_fails_workflow(tmp_path, monkeypatch):
    from rm27.experiments import image_conditions
    from rm27.telemetry import ulog
    workspace = tmp_path / 'scene'
    workspace.mkdir()
    (workspace / 'summary.json').write_text(json.dumps(dict(
        passed=True, input_sha256={}, scene='scene.xml', audit='audit.json')))
    monkeypatch.setattr(simulation, 'doctor', lambda *_: dict(passed=True))
    def completed(*args, **kwargs):
        flight = args[5]
        flight.mkdir()
        (flight / 'summary.json').write_text(json.dumps(dict(
            passed=True, parameters_verified=True, auto_disarm=dict(confirmed=True))))
        (flight / 'test.ulg').touch()
        return 0
    monkeypatch.setattr(image_conditions, 'execute', completed)
    monkeypatch.setattr(ulog, 'analyze', lambda *_: dict(passed=True, fusion={}))
    result = simulation.run(tmp_path / 'run', workspace=workspace)
    assert result['passed'] is False
    assert result['checks']['task'] is True
    assert result['checks']['actual_ev_fusion'] is False


def test_reusing_scene_with_changed_input_refuses_flight(tmp_path, monkeypatch):
    from rm27.experiments import image_conditions
    workspace = tmp_path / 'scene'
    workspace.mkdir()
    changed = tmp_path / 'input.json'
    changed.write_text('{}')
    (workspace / 'summary.json').write_text(json.dumps(dict(
        passed=True, input_sha256={str(changed): 'old-hash'}, scene='scene.xml', audit='audit.json')))
    monkeypatch.setattr(simulation, 'doctor', lambda *_: dict(passed=True))
    def forbidden(*args, **kwargs):
        raise AssertionError('Flight must not start with changed built inputs')
    monkeypatch.setattr(image_conditions, 'execute', forbidden)
    result = simulation.run(tmp_path / 'run', workspace=workspace)
    assert result['passed'] is False
    assert 'Built scene input changed' in result['failure']
