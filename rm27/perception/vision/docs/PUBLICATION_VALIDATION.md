# Repository publication checks — 2026-10-11

User explicitly requested a push and connection/handoff documentation. Publication
baseline: existing remote main `7a91a02e575794ba6756012d31eeb66d73182f35`, reached by
fast-forward from local b8e7299. Older remote OpenVINS work was retained, not replaced.
Local Windows HANDOFF / VAULT_UPDATE were preserved as explicitly versioned archives
before the fast-forward. Current root handoff prepends a portable superseding entry.

## Scope

New connection/resume/index documentation and selected existing `.maixpy` research
source/tests/reports are published. The selection preserves paths and dependency
closure without promoting bench scripts into production or changing behavior.
`.gitattributes` retains research bytes, including recorded report hashes.
`.gitignore` continues excluding unselected generated research content.
Unrelated `tests/camera_capture.py` and `tmp/` are not part of this publication.
No raw datasets/ULogs/UART captures, virtualenvs, vendor/build trees or credentials
are included. No hardware/Vault access, firmware/parameter/baud/wiring change,
calibration, fusion, arm, flight, new acquisition or Stage A/B rerun.

## Fresh offline verification on publication worktree

Root suite:

```powershell
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'
py -3 -m pytest tests -q
```

Exit 0: **226 passed, 1 skipped**, 15.65 s. This checks existing root software,
not real VIO or device connectivity.

Using existing `.maixpy/imu-offline-20261008/venv/Scripts/python.exe`:

| `-m unittest discover -s` directory | `-p` pattern | Passed |
|---|---|---:|
| `.maixpy/m3c-ping-20261009` | `test_heartbeat.py` | 2 |
| `.maixpy/m3c-ping-20261009` | `test_probe.py` | 5 |
| `.maixpy/mavlink2-20261010` | `test_odometry.py` | 2 |
| `.maixpy/imu-input-20261010` | `test_imu_input.py` | 10 |
| `.maixpy/imu-design-20261010` | `test_offline.py` | 9 |
| `.maixpy/imu-design-20261010/followup` | `test_model_review.py` | 7 |
| `.maixpy/imu-transport-20261010` | `test_metrics.py` | 2 |
| `.maixpy/imu-design-20261010/single-uart/reassembly` | `test_ulog_mavlink.py` | 28 |

**65 passed**, each command exit 0. Only offline fixture/library tests executed;
Stage A/B *wire-format tests* are not repetitions of their completed hardware runs.
Saved-file replay is documented historically, not rerun or counted in this task.
Raw saved files are not supplied in Git; a clone cannot claim that replay unaided.

## Publication review

The staged index was exported with `git checkout-index --all --prefix=<new-temp-dir>/`
and the root suite plus all eight research suites rerun from that directory, using
the existing Python environments but only exported repository files. Results:
**226 passed / 1 skipped** for root; **65 passed** for research; both exit 0.
This checks published import/fixture dependencies without relying on untracked
source files. It is not an independent environment installation qualification.

17 local Markdown links in the README/connection/resume entrypoints resolve to
versioned files. Initial whitespace check reported preserved CRLF `.msg` snapshots,
nested patch-context blank lines and Markdown hard breaks. Attributes now explicitly
describe those formats without rewriting frozen evidence; final
`git diff --cached --check` exits 0. A targeted credential-pattern scan over all
staged paths found no matches; this is not a general security certification.
The exported Stage A/B report SHA256 is still
`d9c79727ad749e8ec6ba970a2e41c0ef4aa5ccc3ac6e6e7bc53c0f1bbb7ffd0f`.

The staged-file list and normal push result identify the published artifact.
No force push or history rewrite is planned. Commit/push success is only asserted
after the command completes and the remote main hash is checked. This report does
not pre-record a future successful push or new hardware result.
