"""Small strict-JSON and provenance utilities shared by the offline tools."""
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys


class ExperimentError(ValueError):
    def __init__(self, code, detail):
        self.code = code
        super().__init__(f"{code}: {detail}")


def require(condition, code, detail):
    if not condition:
        raise ExperimentError(code, detail)


def number(value, name, positive=False):
    require(type(value) in (int, float), 'INVALID_NUMBER', name)
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    require(finite and (not positive or value > 0), 'INVALID_NUMBER', name)
    return value


def read_json(path):
    def invalid(value):
        raise ExperimentError('INVALID_JSON', value)
    return json.loads(Path(path).read_text(), parse_constant=invalid)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def object_hash(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def checked_file(path, expected=None):
    path = Path(path).resolve()
    require(path.is_file(), 'DEPENDENCY_MISSING', str(path))
    actual = file_hash(path)
    if expected is not None:
        require(isinstance(expected, str) and expected == actual, 'HASH_MISMATCH', str(path))
    return path, actual


def resolve(base, name):
    require(isinstance(name, str) and bool(name), 'INVALID_PATH', str(name))
    return (Path(base)/name).resolve()


def environment():
    root = Path(__file__).resolve().parents[5]
    git = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, capture_output=True, text=True)
    return dict(python=sys.version, interpreter=sys.executable, system=platform.system(),
                machine=platform.machine(), platform=platform.platform(),
                git_commit=git.stdout.strip() if git.returncode == 0 else 'UNKNOWN',
                modules_sha256={p.name:file_hash(p) for p in sorted(Path(__file__).parent.glob('*.py'))})


# Deliberately private, not a CLI option. All products of this path carry the label.
_TEST_FIXTURE_TOKEN = object()


def fixture_mode(token, *inputs):
    testing = token is _TEST_FIXTURE_TOKEN
    if testing:
        require(all(x.get('purpose') == 'TEST_FIXTURE_ONLY' for x in inputs),
                'FIXTURE_LABEL_REQUIRED', 'all test inputs must be explicitly synthetic')
    else:
        require(all('TEST_FIXTURE_ONLY' not in canonical(x) for x in inputs),
                'TEST_FIXTURE_FORBIDDEN', 'synthetic inputs cannot enter normal experiments')
    return testing
