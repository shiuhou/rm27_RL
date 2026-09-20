"""Download the author's source/model into a new local upstream/ directory.

Uses only the Python standard library. Does not execute downloaded code, modify
system settings, install dependencies, or overwrite an existing checkout.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
import urllib.error
import urllib.request
import zipfile

REPOSITORY = "LGQWakkk/Quadrotor_SE3_Control"
ROOT = Path(__file__).resolve().parent
LIMIT = 128 * 1024 * 1024
HEADERS = {"User-Agent": "SE3-radio-reproduction-starter/1.0", "Accept": "application/vnd.github+json"}


def download(url: str, limit: int = LIMIT) -> bytes:
    request = urllib.request.Request(url, headers=HEADERS)
    chunks = []
    size = 0
    with urllib.request.urlopen(request, timeout=45) as response:
        while True:
            block = response.read(1024 * 1024)
            if not block:
                break
            size += len(block)
            if size > limit:
                raise ValueError("Download exceeds the configured size limit.")
            chunks.append(block)
    return b"".join(chunks)


def extract_archive(content: bytes, destination: Path) -> None:
    """Extract a GitHub archive safely and atomically into an absent directory."""
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(f"Refusing to overwrite {destination}. Use --dest for a new directory.")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(content)) as archive:
        infos = archive.infolist()
        if len(infos) > 10000 or sum(info.file_size for info in infos) > 256 * 1024 * 1024:
            raise ValueError("Archive exceeds the extraction limits.")
        roots = {PurePosixPath(info.filename).parts[0] for info in infos if info.filename}
        if len(roots) != 1:
            raise ValueError("Expected a GitHub ZIP with one top-level repository directory.")
        with tempfile.TemporaryDirectory(prefix="se3-download-", dir=destination.parent) as temporary:
            stage = Path(temporary) / "checkout"
            stage.mkdir()
            written = set()
            for info in infos:
                path = PurePosixPath(info.filename)
                if path.is_absolute() or ".." in path.parts or "\\" in info.filename or ":" in info.filename:
                    raise ValueError("Unsafe archive path.")
                if stat.S_ISLNK(info.external_attr >> 16):
                    raise ValueError("Symlink entries are not accepted.")
                relative = Path(*path.parts[1:])
                if not path.parts[1:]:
                    continue
                target = stage / relative
                if not target.resolve().is_relative_to(stage.resolve()):
                    raise ValueError("Archive entry escapes its destination.")
                if info.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                    continue
                if str(relative).casefold() in written:
                    raise ValueError("Duplicate/case-colliding archive entry.")
                written.add(str(relative).casefold())
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(info) as source, target.open("wb") as output:
                    shutil.copyfileobj(source, output)
            required = ("se3_controller.py", "geometry.py", "motor_mixer.py", "crazyfile/scene.xml")
            if not all((stage / name).is_file() for name in required):
                raise ValueError("This ZIP does not contain the expected author's project layout.")
            stage.rename(destination)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dest", type=Path, default=ROOT / "upstream")
    parser.add_argument("--ref", default="main", help="Branch, tag, or commit SHA; default main")
    parser.add_argument("--zip", type=Path, help="Use an already-downloaded GitHub source ZIP")
    args = parser.parse_args()
    try:
        if args.dest.exists():
            raise FileExistsError(f"{args.dest} already exists; it will not be overwritten.")
        commit = None
        source_url = None
        if args.zip:
            if args.zip.stat().st_size > LIMIT:
                raise ValueError("ZIP exceeds size limit.")
            payload = args.zip.read_bytes()
        else:
            if not re.fullmatch(r"[A-Za-z0-9_.-]+", args.ref):
                raise ValueError("Use a plain branch/tag name or commit SHA.")
            # Resolve once, then download that exact commit; archive hash is also recorded.
            try:
                metadata = json.loads(download(f"https://api.github.com/repos/{REPOSITORY}/commits/{args.ref}",
                                               limit=2 * 1024 * 1024))
                candidate = metadata.get("sha", "")
                if re.fullmatch(r"[0-9a-f]{40}", candidate):
                    commit = candidate
            except (urllib.error.URLError, TimeoutError, ValueError):
                print("GitHub commit metadata unavailable; recording ZIP SHA-256 instead.")
            reference = commit or args.ref
            source_url = f"https://codeload.github.com/{REPOSITORY}/zip/{reference}"
            print(f"Fetching {REPOSITORY} at {reference} ...")
            payload = download(source_url)
        extract_archive(payload, args.dest)
        provenance = {
            "repository": REPOSITORY, "requested_ref": args.ref, "commit": commit,
            "source_url": source_url, "archive_sha256": hashlib.sha256(payload).hexdigest(),
            "retrieved_utc": datetime.now(timezone.utc).isoformat(),
            "source": "local_zip" if args.zip else "github",
        }
        (args.dest / "UPSTREAM_PROVENANCE.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
        print(f"Saved source/model to: {args.dest.resolve()}")
        print("Next: python radio_se3.py demo")
        return 0
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        print(f"ERROR: {exc}")
        print("Alternative: download Code > Download ZIP from the author's GitHub page,")
        print('then run: python fetch_upstream.py --zip "path/to/Quadrotor_SE3_Control-main.zip"')
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
