# Latest VIO-S0 design pointers — 2026-10-10

**2026-10-11 offline implementation:** `single-uart/reassembly/VALIDATION.md`
records the new strict finite ULog/MAVLink reassembler.28 new +19 existing tests
pass; two real saved ULogs in synthetic MAVLink framing reconstruct byte-identically,
preserving391/306 dropout events. Device rate/clock/source completeness remain
UNKNOWN, OpenVINS admission false. Live controller/ACK timing/restoration is the
next software task; `PROPOSED_TRIAL.md` is not executed or approved by this result.

**New constraint and superseding action:** user reports no second available UART.
Read `single-uart/REVIEW.md` first. Spare-UART pin verification is no longer the
next action. Review found existing MAVLink ULog streaming as a complete-record
acquisition prototype before considering custom firmware. Six offline framing/
arithmetic check groups pass; no streaming/capture/configuration was performed.
115200 cannot carry the modeled200Hz complete records; faster baud/software budget
and logger continuity need separate qualification. VIO-S0 remains PARTIAL.
Older dedicated-UART recommendations below are retained historical alternatives.

Read `followup/REPORT.md` together with the frozen `VIO_S0_INPUT_DESIGN.md`.
The follow-up incorporates the completed interface inventory, metadata-inclusive
bandwidth, a pinned-source synchronous DDS time-sync wait hazard, and7 new offline
tests. 92 related tests pass this turn. No hardware touched; VIO-S0 PARTIAL.

USB fallback is blocked on host ACM support/role/management prerequisites, not a
plug-in option. The next approval-gated step is a power-off physical carrier-pin
verification, NOT repeating software inventory. Previous manifest still describes
the frozen original33 files; followup has its own evidence and manifest.
