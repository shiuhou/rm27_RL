"""Read-only DataFlash inventory; no automatic calibration or hardware writes."""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from pymavlink import DFReader


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def inventory(source, output, source_kind):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source_kind not in ('real', 'sitl'):
        raise ValueError('Explicit real or sitl provenance required')
    if source.suffix.lower() != '.bin':
        raise ValueError('Expected original DataFlash .BIN')
    before = digest(source)
    output.mkdir(parents=True, exist_ok=False)
    reader = DFReader.DFReader_binary(str(source))
    counts, parameters, changes, stats, firmware = Counter(), {}, [], {}, []
    first, last = None, None
    selected = {'ATT', 'RATE', 'IMU', 'VIBE', 'BAT', 'RCOU', 'OF', 'RFND', 'MAG', 'XKF1', 'XKF3', 'XKF4'}
    while True:
        message = reader.recv_msg()
        if message is None:
            break
        kind = message.get_type()
        counts[kind] += 1
        values = message.to_dict()
        stamp = values.get('TimeUS')
        if isinstance(stamp, (int, float)):
            first = stamp if first is None else min(first, stamp)
            last = stamp if last is None else max(last, stamp)
        if kind == 'PARM':
            name, value = values.get('Name'), values.get('Value')
            if name in parameters and parameters[name] != value:
                changes.append({'name': name, 'previous': parameters[name], 'value': value, 'TimeUS': stamp})
            if isinstance(value, (int, float)) and math.isfinite(value):
                parameters[name] = value
        if kind == 'MSG' and len(firmware) < 20:
            firmware.append(str(values.get('Message', '')))
        if kind in selected:
            instance = values.get('I', values.get('Instance', values.get('C', 0)))
            for field, value in values.items():
                if field in ('TimeUS', 'mavpackettype') or not isinstance(value, (int, float)):
                    continue
                key = f'{kind}[{instance}].{field}'
                row = stats.setdefault(key, dict(count=0, nonfinite=0, minimum=None, maximum=None))
                if not math.isfinite(value):
                    row['nonfinite'] += 1
                    continue
                row['count'] += 1
                row['minimum'] = value if row['minimum'] is None else min(row['minimum'], value)
                row['maximum'] = value if row['maximum'] is None else max(row['maximum'], value)
    after = digest(source)
    result = dict(passed=before == after and counts.get('FMT', 0) > 0,
                  source=str(source), source_kind=source_kind, source_sha256=before,
                  source_unchanged=before == after, message_counts=dict(counts),
                  first_TimeUS=first, last_TimeUS=last, initial_messages=firmware,
                  parameter_changes=changes, field_ranges=stats,
                  limitations='Inventory only. Ranges retain DataFlash decoded units and instances; no calibration or real-flight pass implied.',
                  missing_message_types=sorted(selected-set(counts)))
    (output/'parameters.json').write_text(json.dumps(parameters, indent=2, allow_nan=False))
    (output/'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-kind', choices=['real', 'sitl'], required=True)
    args = parser.parse_args()
    result = inventory(args.source, args.output, args.source_kind)
    print(json.dumps({k: result[k] for k in ('passed', 'source_kind', 'source_sha256', 'source_unchanged', 'missing_message_types')}))
    raise SystemExit(0 if result['passed'] else 1)
