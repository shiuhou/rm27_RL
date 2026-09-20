import json
from rm27.experiments.analyze_route import analyze


def test_analysis_exports_native_json_and_rejects_contact(tmp_path):
    run = tmp_path/'run'
    run.mkdir()
    (run/'metadata.json').write_text(json.dumps({'route_audit': {
        'waypoints_world_m': [[0, 0, 1], [1, 0, 1]], 'tracking_margin_m': .08}}))
    (run/'summary.json').write_text(json.dumps({'passed': True, 'final_local_position': [0, 0, .02]}))
    row = {'phase': 'ROUTE', 'truth': {'position_nwu': [.5, 0, 1], 'velocity_nwu': [.05, 0, 0]},
           'state_sent': {'timestamp': 1}, 'contacts': [{'geoms': ['uav', 'wall']}]}
    (run/'frames.jsonl').write_text(json.dumps(row)+'\n')
    report = analyze(run, tmp_path/'output')
    assert report['passed'] is False
    assert json.loads((tmp_path/'output/summary.json').read_text())['route_contact_frames'] == 1
