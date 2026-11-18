"""Source-grouped Fixed A continuation labels for the R7 auxiliary pilot."""
from pathlib import Path
import json
import numpy as np
from scipy.optimize import minimize
from scipy.special import expit
from .asset import ROOT, digest, read_json, write_result
from .r4_potential_audit import inputs, auc
from .flight_suite import rotation_matrix

OUT = ROOT / '.agents/runs/asm2-terminal/r7'


def build():
    cfg, manifest, rows, task, profile, spec, _, _ = inputs()
    goal = spec.target_center + (-spec.patch_face_z_m-task['goal_inset_m'])*spec.target_normal
    records = []
    pins = {}
    for row in rows:
        records.append(dict(source=row['case'], identity=f"phase-r/{row['case']}/{row['frame']}/{row['variant']}",
                            state=row['state'], motors=row['motor_thrust_N'], targets=row['motor_target_N'],
                            label=int(row['outcome']['qualified']), horizon=manifest['horizon_s']))
    for phase in ('r5', 'r5-1'):
        folder = OUT.parent / phase
        frozen = read_json(folder/'manifest.json')
        for path, expected in frozen['pins'].items():
            if digest(ROOT/path) != expected:
                raise ValueError(f'Changed evidence {path}')
        for path in sorted((folder/'cases').glob('*/outcome.json')):
            outcome = read_json(path)
            data_path = path.parent/'physics.npz'
            with np.load(data_path) as data:
                records.append(dict(source=outcome['case'], identity=f'{phase}/{outcome["id"]}',
                                    state=data['states'][0].tolist(), motors=data['motor_thrust_N'][0].tolist(),
                                    targets=data['motor_target_N'][0].tolist(), label=int(outcome['result']['qualified']),
                                    horizon=frozen['horizon_s']))
            for p in (path, data_path, folder/'manifest.json'):
                pins[str(p.relative_to(ROOT))] = digest(p)
    for name in ('manifest.json', 'results.json'):
        p = ROOT/cfg['archive']/name
        pins[str(p.relative_to(ROOT))] = digest(p)
    states = np.array([r['state'] for r in records])
    labels = np.array([r['label'] for r in records])
    sources = np.array([r['source'] for r in records])
    observation = np.column_stack((np.tile(goal-states[:, :3], (1, 10)), states[:, 7:10],
                                  rotation_matrix(states[:, 3:7]).transpose(0, 2, 1).reshape(-1, 9), states[:, 10:13])).astype(np.float32)
    physical = np.column_stack((observation, [r['motors'] for r in records], [r['targets'] for r in records]))
    if len(set(r['horizon'] for r in records)) != 1:
        raise ValueError('Mixed label horizons require explicit treatment')
    comparisons = {}
    for name, features in [('actor_visible', observation), ('full_physical', physical)]:
        predictions = np.zeros(len(records))
        folds = []
        for source in sorted(set(sources)):
            train = sources != source
            valid = ~train
            mean, scale = features[train].mean(0), features[train].std(0)
            scale = np.maximum(scale, 1e-6)
            x = np.column_stack(((features-mean)/scale, np.ones(len(features))))
            # One fixed ridge logistic model. Equal total weight per source.
            weights = np.array([1/sum(sources[train] == s) for s in sources[train]])
            weights /= weights.sum()
            def objective(beta):
                z = x[train] @ beta
                loss = np.sum(weights*(np.logaddexp(0, z)-labels[train]*z)) + .01*np.sum(beta[:-1]**2)/2
                gradient = x[train].T @ (weights*(expit(z)-labels[train]))
                gradient[:-1] += .01*beta[:-1]
                return loss, gradient
            fit = minimize(objective, np.zeros(x.shape[1]), jac=True, method='L-BFGS-B', options={'maxiter': 500})
            p = expit(x[valid] @ fit.x)
            predictions[valid] = p
            folds.append(dict(source=source, n=int(valid.sum()), positives=int(labels[valid].sum()),
                              auc=auc(labels[valid], p), brier=float(np.mean((p-labels[valid])**2)),
                              accuracy=float(np.mean((p >= .5)==labels[valid])), converged=bool(fit.success)))
        comparisons[name] = dict(folds=folds, auc=auc(labels, predictions),
                                 brier=float(np.mean((predictions-labels)**2)), predictions=predictions.tolist())
    OUT.mkdir(exist_ok=True)
    with (OUT/'supervision.npz').open('xb') as f:
        np.savez_compressed(f, observations=observation, physical=physical, labels=labels, sources=sources)
    write_result(OUT/'supervision.json', dict(records=records, pins=pins, comparison=comparisons,
        count=len(records), positive=int(labels.sum()), independent_sources=len(set(sources)),
        scope='Actual Fixed A reset continuations only. Correlated frames and perturbations grouped by original source. Common remaining horizon. No inferred suffix or planner labels.',
        classifier='Fixed ridge logistic regression, lambda .01, leave one source out. No hyperparameter search.',
        training_transitions=0))
    print(json.dumps({k: v for k,v in comparisons.items()}, indent=2))

if __name__ == '__main__':
    build()
