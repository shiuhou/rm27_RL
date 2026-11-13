import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from rm27_asm2 import regional_update_runtime as runtime


class RuntimeAliasTests(unittest.TestCase):
    def test_only_same_image_alias_is_accepted(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            name = 'configs/rm27_asm2_collision.json'
            source = root/name
            source.parent.mkdir()
            before = json.dumps(dict(isaac_image='old-tag', solver='PGS'))
            source.write_text(before.replace('old-tag', 'new-tag'))
            frozen = dict(source_pins={name: hashlib.sha256(before.encode()).hexdigest()},
                          historical_source_pins={}, runtime=dict(image='old-tag', image_id='sha256:same'))
            (root/'freeze.json').write_text(json.dumps(frozen))
            with patch.object(runtime, 'ROOT', root), patch.object(runtime.experiment.workflow, 'verify_freeze', return_value=frozen), patch.object(runtime.subprocess, 'check_output', return_value='sha256:same\n'):
                self.assertEqual(runtime.verify_alias({}, root), frozen)
                receipt = json.loads((root/'runtime-alias-receipt.json').read_text())
                self.assertFalse(receipt['training_change'])
                self.assertEqual(receipt['image_id'], 'sha256:same')
                source.write_text(source.read_text().replace('PGS', 'TGS'))
                with self.assertRaisesRegex(ValueError, 'beyond image alias'):
                    runtime.verify_alias({}, root)
                source.write_text(before.replace('old-tag', 'new-tag'))
                with patch.object(runtime.subprocess, 'check_output', return_value='sha256:different\n'):
                    with self.assertRaisesRegex(ValueError, 'different runtime'):
                        runtime.verify_alias({}, root)
                frozen['source_pins']['protected.txt'] = 'wrong'
                (root/'protected.txt').write_text('changed')
                with self.assertRaisesRegex(ValueError, 'Frozen input changed'):
                    runtime.verify_alias({}, root)


if __name__ == '__main__':
    unittest.main()
