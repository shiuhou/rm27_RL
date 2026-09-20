# Local references

This directory keeps user-provided reference material that is used by the
integration documents but is not redistributed in Git.

- `rules/`: local RoboMaster rulebook PDFs; their recorded SHA-256 values are
  tracked in [`docs/RULEBOOK_BASELINE.md`](../docs/RULEBOOK_BASELINE.md).
- `images/`: source photographs and other visual references used for asset
  audits.
- `cad_raw/`: local STEP/SLDASM sources for the dock, aircraft and base audits;
  the active files are excluded from Git and their hashes are recorded in the
  corresponding reports. `cad_raw/retired/` contains the retired whole-field
  source and is kept only for historical comparison.

The files in these subdirectories are intentionally ignored by Git.
