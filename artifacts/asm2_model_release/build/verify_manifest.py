"""Verify every payload path, file hash and the release content ID."""

import hashlib
import json
from pathlib import Path


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def verify(root):
    manifest = json.loads((root / "manifest.json").read_text())
    if manifest["schema"] != "asm2_model_release_v1":
        raise ValueError("Unknown release manifest")
    files = manifest["files"]
    payload = json.dumps(files, sort_keys=True, separators=(",", ":")).encode()
    if hashlib.sha256(payload).hexdigest() != manifest["payload_sha256"]:
        raise ValueError("Release content ID mismatch")
    paths = set()
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"Unexpected symlink: {path}")
        if path.is_file() and path != root / "manifest.json":
            paths.add(path.relative_to(root).as_posix())
    if paths != files.keys():
        raise ValueError(f"Missing payload paths: {sorted(files.keys()-paths)}; extra: {sorted(paths-files.keys())}")
    for relative, expected in files.items():
        if digest(root / relative) != expected:
            raise ValueError(f"File hash mismatch: {relative}")
    print(f"{manifest['version']} {len(files)} verified files, payload SHA-256 {manifest['payload_sha256']}")


if __name__ == "__main__":
    verify(Path(__file__).resolve().parent)
