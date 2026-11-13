"""Resume the frozen experiment after an external Docker image alias rename.

No frozen source or manifest is rewritten. Accept ONLY an image-tag substitution
that reconstructs the original configuration byte-for-byte and resolves to the
same immutable Docker image. All training continues by its original image ID.
"""
import hashlib
import json
import subprocess
from unittest.mock import patch

from . import regional_update_experiment as experiment
from .asset import ROOT, digest, read_json, write_result


def verify_alias(cfg, output):
    frozen = experiment.workflow.verify_freeze(cfg, output)
    for name, expected in {**frozen['historical_source_pins'], **frozen['source_pins']}.items():
        actual = digest(ROOT/name)
        if actual == expected:
            continue
        if name != 'configs/rm27_asm2_collision.json':
            raise ValueError(f'Frozen input changed: {name}')
        raw = (ROOT/name).read_text()
        alias = json.loads(raw)['isaac_image']
        original = raw.replace(json.dumps(alias), json.dumps(frozen['runtime']['image']))
        if hashlib.sha256(original.encode()).hexdigest() != expected:
            raise ValueError('Runtime change extends beyond image alias')
        receipt_path = output/'runtime-alias-receipt.json'
        metadata = dict(config=name, before_sha256=expected, after_sha256=actual,
                        original_image=frozen['runtime']['image'], alias=alias,
                        image_id=frozen['runtime']['image_id'],
                        adapter_sha256=digest(__file__), freeze_sha256=digest(output/'freeze.json'),
                        training_change=False, rationale='Externally renamed tag; identical image and configuration semantics.')
        # Only the host has Docker access. The child verifies the host's receipt
        # and is launched by the original immutable image ID, never the new tag.
        if str(ROOT) != '/sf':
            image_id = subprocess.check_output(experiment.docker_prefix()+[
                'image', 'inspect', alias, '--format', '{{.Id}}'], text=True).strip()
            if image_id != frozen['runtime']['image_id']:
                raise ValueError('New alias resolves to a different runtime')
            if not receipt_path.exists():
                write_result(receipt_path, metadata)
        if read_json(receipt_path) != metadata:
            raise ValueError('Runtime alias receipt changed')
    return frozen


def main():
    original_command = experiment.docker_command

    def launch(*args):
        command = original_command(*args)
        index = command.index('rm27_asm2.regional_update_experiment')
        command[index] = 'rm27_asm2.regional_update_runtime'
        return command

    with patch.object(experiment, 'verify_freeze', verify_alias), patch.object(experiment, 'docker_command', launch):
        experiment.main()


if __name__ == '__main__':
    main()
