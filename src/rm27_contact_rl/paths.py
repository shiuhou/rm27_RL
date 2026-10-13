"""Configuration roots for a complete editable RL checkout, independent of cwd."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if not (ROOT / "configs/contact_rl/minimal.json").is_file():
    raise RuntimeError("Install rm27-contact-rl editable from a complete checkout with configs/")
