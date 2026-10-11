"""Read numerical-failure matrices without changing the original evidence."""
import argparse
import json
from pathlib import Path

import numpy as np

from rm27.perception.sitl.dataset import digest


def analyze(recording, output):
    recording, output = Path(recording).resolve(), Path(output).resolve()
    matrices = {name:np.loadtxt(recording/f'covariance_{name}.csv',delimiter=',',ndmin=2)
                for name in ['prior','H','R','S','posterior']}
    P, H, R, S = [matrices[k] for k in ['prior','H','R','S']]
    checks = dict(finite=all(np.isfinite(v).all() for v in matrices.values()),
        dimensions=H.shape[1]==P.shape[0] and R.shape==S.shape==(H.shape[0],H.shape[0]))
    spectra = {}
    for name in ['prior','R','S','posterior']:
        matrix = matrices[name]
        eig = np.linalg.eigvalsh((matrix+matrix.T)*.5)
        roundoff = 64*len(eig)*np.finfo(float).eps*max(1.,abs(matrix).max())
        spectra[name] = dict(eigen_minimum=float(eig[0]),eigen_maximum=float(eig[-1]),
            diagonal_minimum=float(matrix.diagonal().min()),symmetry_error=float(abs(matrix-matrix.T).max()),
            declared_roundoff_bound=float(roundoff),positive_within_roundoff=bool(eig[0]>=-roundoff))
    consistency = float(abs(S-(H@P@H.T+R)).max())
    K = np.linalg.solve(S,H@P).T
    residual = np.eye(P.shape[0])-K@H
    candidate = residual@P@residual.T+K@R@K.T
    candidate = (candidate+candidate.T)*.5
    result = dict(passed=all(checks.values()),checks=checks,spectra=spectra,
        innovation_condition_number=float(np.linalg.cond(S)),innovation_consistency_maximum=consistency,
        independent_joseph_eigen_minimum=float(np.linalg.eigvalsh(candidate)[0]),
        independent_joseph_diagonal_minimum=float(candidate.diagonal().min()),
        input_sha256={str(recording/f'covariance_{k}.csv'):digest(recording/f'covariance_{k}.csv') for k in matrices},
        run_type='read-only native covariance diagnostic',flight_qualified=False,
        scope='Diagnostic validity only; passed is not estimator, covariance repair or flight acceptance. No input or eigenvalue is clipped.')
    output.mkdir(parents=True,exist_ok=False)
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    raise SystemExit(0 if analyze(args.recording,args.output)['passed'] else 1)
