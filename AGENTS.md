# RM27 Contact RL repository instructions

Read `.agents/docs/SPEC.md` for adopted engineering conventions and research
contracts, and `.agents/docs/DESIGN.md` for implementation and supported commands.
These documents are self-contained; external RMRL drafts are source material only.

- This repository owns Contact RL and `packages/physics`, the canonical Physics
  package. Physics must not import RL, Gymnasium, Torch or the parent RM27 tree.
- Keep reviewed values in `configs/`, reusable checks in `tests/`, and generated
  results/checkpoints in ignored `artifacts/`. Do not create empty scaffolding.
- Use a checkout-specific `.venv` and the single third-party hash lock. Never
  modify shared environments or other workspaces.
- Preserve dynamics, contact scoring, timing, action and checkpoint semantics
  unless their change is explicitly in scope. Engineering estimates are not
  hardware calibration. Report engineering checks and task success separately.
- Read current Git state and relevant callers before changing code. Complete
  authorized implementation and proportionate checks; do not expand to training,
  hardware or servers without the corresponding task authorization.
- Preserve existing evidence. New runs need new identifiers; do not infer PASS
  from a directory name. Report actual PASS/FAIL/NOT_RUN/BLOCKED and limitations.
- Maintain only the user's existing contact_rl_01 activity plan at its existing
  external path. Do not copy it into this repository or create parallel reports.
- S1/S2 separation is authorized; S3 original-RM27 consumer adaptation, default
  branch changes, merge, force-push and new PPO training are not part of it.
- Explicit user instructions take precedence over local workflow preferences.
