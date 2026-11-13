"""Install checksum-pinned official packages in the derived image only."""

import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile


DOWNLOADS = Path('/tmp/isaac-repair')
EXTENSIONS = Path('/isaac-sim/extscache')
ML_ARCHIVE = EXTENSIONS / 'omni.isaac.ml_archive-2.1.2+106.5.0.lx64.cp310/pip_prebundle'


def verified(name, expected):
    path = DOWNLOADS / name
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError(f'Unexpected package hash: {name}')
    return path


def install_warp():
    source = verified('warp-source.tar.gz', '8cc3511c71b06d51cc48f872fb1136a2742c876b7b354c5b5b7c1f79bc58649e')
    wheel = verified('warp_lang-1.8.1-py3-none-manylinux_2_28_x86_64.whl',
                     'cfc59e1070ad71531b5d83186de48162507277af344a102fa33d5df9cdb942f7')
    subprocess.run([sys.executable, '-m', 'pip', 'install', '--no-cache-dir', '--no-index',
                    '--no-deps', '--force-reinstall', str(wheel)], check=True)
    with tarfile.open(source) as archive:
        for member in archive.getmembers():
            if not member.isfile():
                continue
            for extension in ('omni.warp.core', 'omni.warp'):
                prefix = f'warp-1.8.1/exts/{extension}/'
                if not member.name.startswith(prefix):
                    continue
                relative = Path(member.name[len(prefix):])
                if relative.is_absolute() or '..' in relative.parts:
                    raise ValueError(member.name)
                destination = EXTENSIONS / f'{extension}-1.8.1' / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as stream, destination.open('wb') as output:
                    shutil.copyfileobj(stream, output)
    # Kit loads its extension payload ahead of site-packages. Upgrade both.
    import warp
    shutil.copytree(Path(warp.__file__).parent, EXTENSIONS / 'omni.warp.core-1.8.1/warp')
    shutil.rmtree(EXTENSIONS / 'omni.warp.core-1.5.1+lx64')
    shutil.rmtree(EXTENSIONS / 'omni.warp-1.5.1')
    source.unlink()
    wheel.unlink()


def install_torchvision():
    wheel = verified('torchvision-0.20.1+cu118-cp310-cp310-linux_x86_64.whl',
                     '2a0b9fee398ed0dba1cee48dee31d80981420393f8b5c7980e273eb85fc6b9cb')
    # Replace the whole matching wheel, including companion shared libraries.
    for name in ('torchvision', 'torchvision.libs', 'torchvision-0.20.1+cu118.dist-info'):
        shutil.rmtree(ML_ARCHIVE / name)
    subprocess.run([sys.executable, '-m', 'pip', 'install', '--no-cache-dir', '--no-index',
                    '--no-deps', '--target', str(ML_ARCHIVE), str(wheel)], check=True)
    wheel.unlink()


if __name__ == '__main__':
    {'warp': install_warp, 'torchvision': install_torchvision}[sys.argv[1]]()
