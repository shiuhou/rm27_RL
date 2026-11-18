"""Independent native endpoint assessment for the R7 pilot."""
from unittest.mock import patch
from .asset import read_json
from .r7_experiment import configuration
from . import r3_report, r6_report


def main():
    cfg, _, _, out = configuration()
    decision = r6_report.assessment(out, 98304)
    if decision['extension_gate_pass']:
        print('EXTENSION GATE PASSED. Evaluate the authorized matched second block before final selection.')
        return
    with patch.object(r3_report, 'assessment', r6_report.assessment):
        r3_report.final(cfg, out, 98304)
    print(decision)

if __name__ == '__main__':
    main()
