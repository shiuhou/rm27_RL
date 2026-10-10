# VSL-0B Localization Contract Foundation

`LocalizationEstimate` v1 is the self-localization product. It is separate
from `TargetEstimate`, which describes an external target relative to the
vehicle. No estimator or ExternalNav adapter is included here.

`T_parent_child` transforms coordinates expressed in `child_frame` into
`parent_frame`. The record names both frames and does not carry a per-message
direction flag. The repository has no complete localization frame registry;
frame IDs therefore remain explicit data and must be reviewed before control
use.

Source and publish times retain raw value, unit, clock domain, semantic, and
evidence status. Freshness is computed by a consumer when clock domains permit;
there is no serialized immutable measurement age. Unknown VIN PTS values remain
unknown and are never relabeled as exposure timestamps.

`localization_epoch` increments whenever continuity is not guaranteed: reset,
new origin, map replacement, or estimator restart. `RESET` is an explicit
event, while `map_id` identifies map provenance. Schema validity, estimator
tracking state, freshness, and future flight eligibility are separate checks.
Covariance and quality are optional and are never fabricated.

The manifest validator accepts unknown timing and absent optional IMU or
uncertainty, while rejecting malformed records, non-finite values, impossible
dimensions/strides, bad sequence order, missing calibration files, hash
mismatches, and unverified exposure claims.

## 2026-09-22 version 2 update

The v1 description above is historical. The v2 contract and timestamp-claim
policy supersede it; see [VSL-0B.1 preparation update](VSL-0B1_CALIBRATION_PREP.md).
Unverified exposure claims are now preserved as claims, and file/hash validation
is explicitly separated from dataset qualification.
