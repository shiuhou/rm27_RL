# RM27 Agile Flight RL repository instructions

Read `.agents/docs/SPEC.md` for adopted engineering conventions, current research
scope and acceptance; read `.agents/docs/DESIGN.md` for implementation and proposals.
The repository documents are self-contained. `engineering.md` is the user-named
source convention, not an additional independently maintained specification.

`AGENTS.md` remains the root work entry. `.agents/docs/SPEC.md` and
`.agents/docs/DESIGN.md` are the only maintained specification and design;
do not create parallel root-level copies. `I4_ASTRA_PLAN_PROMPT.md` is historical
reference input, not an additional authority or an instruction to replace
adopted decisions. Preserve user changes when merging inputs.

Preserve later explicit user decisions over earlier draft suggestions.
Distinguish user-confirmed goals, proposed parameters, adopted implementations
and observed results; a plan's restatement alone is not evidence of confirmation.

- This repository owns Agile Flight RL and the canonical `packages/physics` package.
  Physics must not import RL, Gymnasium, Torch or the parent RM27 working tree.
- The research goal is autonomous policy discovery for fixed-target agile flight,
  then contact, separation and recovery. Teacher/Student imitation is not a
  required architecture, reward, warm start or prerequisite.
- I0—I3 and separation S1/S2 are existing baselines. I4 now targets no-contact
  agile terminal arrival, not a rewrite of historical contact-recovery success.
  Preserve old task defaults, scoring, configurations and checkpoint compatibility.
- Do not rebuild the project under rm27_drones, rename packages just to match the
  research title, or recreate repositories, environments and audit frameworks.
- Keep configuration in `configs/`, reusable checks in `tests/`, and actual
  generated results/checkpoints in ignored `artifacts/`. No empty scaffolding.
- Use this checkout's isolated environment and one third-party hash lock.
  Do not modify shared venvs, unrelated workspaces or the original RM27 consumers.
- Confirm actual code, callers, state and source before changes. I4 implementation
  may make minimal necessary decoupling only when separately authorized; never
  loosen historical scoring or disable safety geometry to obtain a PASS.
- Keep USER-CONFIRMED research direction, PROPOSED design, ADOPTED implementation
  and OBSERVED results distinct. Engineering estimates are not hardware calibration.
- Maintain one current activity plan per task. I4 uses
  `.agents/plans/plan_agile_terminal_arrival_01.md`; update that file across
  iterations and debugging, without creating 02/03 plans or parallel reports.
  The former external contact_rl_01 plan is historical and no longer active.
  The plan is ignored by Git and is not distributed with the repository.
  Formal requirements and designs remain self-contained in the tracked docs;
  reading the local plan is not required to understand them.
- Start every activity plan with a "§0 progress and handoff" block: the single place for
  task progress, ordered todo items (each with a runnable command and a done criterion),
  repository state, key results and active authorization (format in SPEC §3). Update it at
  milestones, before handoff or interruption, and when authorization changes. A new session
  resumes from that block; do not create separate processing, status, todo or handoff files.
- The user has authorized I4 implementation using the current local plan:
  terminal-arrival integration, focused checks, one bounded CPU PPO run and its
  fixed evaluation set. That batch is completed; preserve its results, Contact
  defaults and existing checkpoints. Do not repeat or extend its training budget
  without a new user instruction. No Git publication, S3, server or hardware
  operations are authorized by this batch.
- Authorization remains valid for the explicitly approved scope. Approval of
  I0—I3 or S1/S2 does not authorize I4 implementation, S3 or default-branch changes.
  I4 authorization comes from the user's explicit implementation request, not
  from prior document reviews or the plan file itself.
- Preserve evidence and use new run identifiers. Report PASS/FAIL/NOT_RUN/BLOCKED,
  task success and research uncertainty separately. Proportionate checks, not
  repeated full regression, are the default for approved implementation batches.
- Explicit user instructions and applicable higher-priority instructions govern
  scope; neither a draft nor a completed plan grants implementation authority.

## Common engineering standard

Apply `.agents/docs/engineering.md` for the shared engineering process. This
file remains the RL-specific adapter: it defines package ownership, research
scope, evidence boundaries and authorization. Keep the common file synchronized
with the RM27 integration repository when the shared standard changes; do not
copy RL-specific requirements into the common file.
