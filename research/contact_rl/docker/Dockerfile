# Cached official amd64 base; digest pins the actual image, not just the tag.
FROM ubuntu:24.04@sha256:33ceb71981b602c1a7443a53469e4dba065f7503eab3078a2d7a57a2ab987517

ENV PATH=/opt/venv/bin:$PATH \
    VIRTUAL_ENV=/opt/venv \
    PYTHONPATH="" \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    MUJOCO_GL=disable \
    HOME=/tmp \
    CONTACT_RL_BUILD_SOURCE=/opt/contact-rl/build-source.json

# CPU/headless MuJoCo and PPO: no Git, compiler, CUDA or flight software.
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-venv ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv /opt/venv

WORKDIR /workspace
COPY pyproject.toml requirements-contact-rl.lock ./
RUN python -m pip install --no-cache-dir --only-binary=:all: --require-hashes \
    --no-deps -r requirements-contact-rl.lock

# COPY uses the whitelist and the real checkout, not git archive HEAD.
COPY . .
ARG SOURCE_GIT_JSON
RUN <<'BUILD_SOURCE'
python - <<'PY'
import hashlib
import json
import os
from pathlib import Path

root = Path('/workspace')
identity = json.loads(os.environ['SOURCE_GIT_JSON'])
files = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
         for path in sorted(root.rglob('*')) if path.is_file()}
allowed = {line[1:] for line in (root / '.dockerignore').read_text().splitlines()
           if line.startswith('!') and not line.endswith('/')}
if set(files) != allowed:
    raise RuntimeError(f'Build context differs from whitelist: '
                       f'extra={sorted(set(files) - allowed)}, '
                       f'missing={sorted(allowed - set(files))}')
print(f'Build context: {len(files)} whitelisted files, '
      f'{sum((root / name).stat().st_size for name in files)} bytes')
dockerfile = (root / 'research/contact_rl/docker/Dockerfile').read_text()
base = next(line.split()[1] for line in dockerfile.splitlines() if line.startswith('FROM '))
identity.update(base_image=base, included_files_sha256=files,
                source_snapshot_sha256=hashlib.sha256(
                    json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
                dependency_lock_sha256=files['requirements-contact-rl.lock'])
output = Path(os.environ['CONTACT_RL_BUILD_SOURCE'])
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(identity, indent=2) + '\n')
PY
BUILD_SOURCE
RUN python -m pip install --no-deps --no-build-isolation --no-index -e .

# The launcher uses the caller's UID:GID; direct runs also default to non-root.
USER 65532:65532
WORKDIR /tmp
ENTRYPOINT ["python", "-m", "rm27_contact_rl"]
CMD ["--help"]
