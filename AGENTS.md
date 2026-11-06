# RM27 Agile Flight RL repository instructions

## RK3576 reference documents

For hardware or deployment work in the RM27 parent checkout, first read
`../../docs/REAL_HARDWARE.md`, section "RK3576 vendor references (DDR5 only)";
raw manuals are in `../../RK3576/`. Use only LPDDR5 CV00 / MV12 references,
not LPDDR4 variants. These local manuals are not bundled with standalone clones;
if absent, report the missing references and do not guess board-specific values.
The selected documents do not establish the installed revision or hardware qualification.

## PDF-aligned current scope (2026-10-30)

Research source: `RM27 无人机组开发文档 (2).pdf`, SHA-256
`d4526f85248623ada0a7c64e120184c8b6838dae215593ea034ee6e8825c437e`. Current goal: repeated fixed-base underside contact,
separation, attitude/estimation recovery and target reacquisition in one sortie.
RK3576 is the new platform direction; its full hardware/data chain is not qualified.
Return-to-dock and precision landing below belong to historical baselines, not the
new season's mandatory deliverables. Traditional/RL allocation remains a research
choice. Detailed algorithms/hardware tasks are scheduled separately by the user.

Component responsibility: Autonomous agile-flight/contact policy research and fair traditional-control comparison. First Contact is a research direction, not a completed capability or permanent policy allocation. This batch only exposes existing observation/action conversion and fixed-seed untrained actor inference; no learning.

The authorized cross-repo interface batch is synthetic/offline only, with no
learning, firmware execution, hardware operations or Git publication. The root
repository coordinates the single activity plan and derived report. Component
contracts remain self-contained; the shared engineering method is unchanged.

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

- This repository owns Agile Flight RL. The canonical Physics package lives in
  `rm_uav_mujoco/packages/physics` and is consumed as an external dependency.
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
- Keep the plan's "§0 progress and handoff" block as defined in the shared
  `engineering.md` (rule 3); a new session resumes from it.
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


## Shared real-camera test videos

Read `data/shared_vision/README.md` and its linked manifest before using the
shared real-camera MP4 fixtures. This local relative link points to the RM27
integration workspace's `data/vision/` catalog; raw videos are not bundled in
Git clones. Keep inputs unchanged and derived results in this repo's ignored
artifacts. Encoded video alone is not synchronized Camera+IMU VIO or verified
sensor/exposure timing evidence. If the link/data is absent, obtain or relink
the shared dataset instead of inventing missing provenance.

## Shared M3C and MaixCAM2 board references

For M3C/MaixCAM2 carrier, wiring or mechanical reference work, read
`data/shared_m3c_board/README.md` and `data/shared_m3c_board/manifest.json`;
source files are under `data/shared_m3c_board/raw/`. This relative local link
resolves to the integration repo's single hardware reference collection. Match
378C/379C/3.0 documents to the actual board; filenames do not verify installed
revision, IMU wiring or camera synchronization. Keep sources unchanged and
derivatives in this repo's artifacts. Standalone clones need the payloads and a
new local link; report missing data.
