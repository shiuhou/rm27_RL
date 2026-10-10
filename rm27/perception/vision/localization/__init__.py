"""Versioned self-localization contracts and experiment metadata validation.

This package deliberately contains no estimator or flight-control integration.
"""

from .schema import (
    LOCALIZATION_SCHEMA_VERSION,
    LocalizationEstimate,
    TimestampEvidence,
    TrackingState,
    ScaleState,
    LocalizationStreamValidator,
)
from .manifest import ManifestError, validate_manifest, load_manifest

__all__ = [
    "LOCALIZATION_SCHEMA_VERSION",
    "LocalizationEstimate",
    "TimestampEvidence",
    "TrackingState",
    "ScaleState",
    "LocalizationStreamValidator",
    "ManifestError",
    "validate_manifest",
    "load_manifest",
]
