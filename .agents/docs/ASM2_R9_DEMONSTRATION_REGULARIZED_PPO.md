# ASM2 R9 — Controlled demonstration-regularized PPO

**COMPLETE — Scientific No-Go; baseline promotion No-Go.** The authorized single arm completed exactly **393,216 new transitions /384 updates** and **726 native evaluations/replay reproductions**, within the831 cap. R8 was neither retrained nor reevaluated. No extension, hardware work, commits or pushes occurred. Keep Phase4 as the verified learned baseline and Fixed A as the continuation reference.

The regularizer improved late action imitation relative to archived R8 B and sometimes preserved more braking, but it did not produce persistent, multi-anchor qualified earlier contact. Every demonstrated-start checkpoint remained0/3. The only earlier gains were two variants of one already-favorable anchor at196608, lost by294912 and final. Lower imitation error and more frequent contact did not establish skill bridging.

## Preregistered method and execution integrity

The [frozen specification](R9_EXPERIMENT_SPEC.md), [preflight report](R9_PREFLIGHT_REPORT.md) and [comparison plan](R9_COMPARISON_PLAN.md) remain unchanged historical records of the approval gate. The user subsequently issued `APPROVE_R9_393216`; execution authorization and source/runtime pins are in [R9 freeze](../runs/asm2-terminal/r9/freeze.json) and [authorization](../runs/asm2-terminal/r9/authorization.json).

R9 started from the same saved BC Actor, Critic, ValueNorm and both inherited PPO Adam states as R8 B. All five initial state dictionaries match exactly, as do every recorded tensor in the first pre-update rollout. Seed42722, original resets,45-element observations, Direct-CTBR actions, plant, reward, GAE, PPO ratio/value objectives and native scorer stayed fixed. One charged1024-transition save/restart matched R8. The Actor received the fixed full-dataset current-Gaussian NLL regularizer before original clipping/Adam, λ=.01 linearly decaying with completed new transitions. No coefficient or checkpoint was selected using fresh results.

Only isolated R9 orchestration/evaluation files and execution configuration were added. The base trainer file was unchanged; a scoped wrapper replaced its legacy collapse predicate with the preregistered two-assessment zero-braking/zero-terminal predicate. The original preflight-tested regularizer remained unchanged throughout execution.

One disclosed [integration amendment](../runs/asm2-terminal/r9/integration-amendment.json), with a recoverable [source diff](../runs/asm2-terminal/r9/integration-amendment.patch), corrected fresh replay ordering before any fresh evaluation/replay: newly qualified earlier anchors first, then unrepresented rejection reasons, then lexicographic remainder. It also added explicit verification of these two amended integration-file hashes. Original freeze was preserved. No loss, policy, data, plant, scorer, checkpoint-selection rule or budget changed. A small regression case verified that an earlier success precedes a braking success and that an unrepresented failure is retained.

## Matched native development results

| Additional transitions | R8 B earlier /36 | R9 earlier /36 | R8 B robust /12 | R9 robust /12 | R8 B braking /36 | R9 braking /36 | R9 terminal /30 |
|---:|---:|---:|---:|---:|---:|---:|---:|
|0, reused BC|0|0|0|0|33|33|30|
|98304|0|0|0|0|29|27|30|
|196608|0|2|0|0|19|22|30|
|294912|3|0|1|0|23|30|30|
|393216|0|0|0|0|23|25|30|

R8 B terminal also remains30/30 throughout. No R9 trained checkpoint reaches braking≥32/36, so none is eligible for promotion. The frozen fallback selects final393216 for analysis only. [Selection was sealed](../runs/asm2-terminal/r9/final-selection.json) before fresh evaluation; selected and final share SHA256 `091d9c8f00e30d202b3ee59656c5c5b04ad2bfd759cf53624b83658267a41562`.

Earlier source independence:

| Original source family | Distinct anchors | Trials per checkpoint | R9 qualified at98304 /196608 /294912 /393216 |
|---|---:|---:|---|
|dev_far|6|18|0 /0 /0 /0|
|dev_lateral_positive|1|3|0 /2 /0 /0|
|stationary_1|5|15|0 /0 /0 /0|

The two qualifications are nominal and minus of `dev_lateral_positive:730`; plus fails. This is one partial anchor, not two skills. R8's transient3/36 also belongs to this historically favorable anchor. Correlated frames within a source are not independent families. Full identities, variants and outcomes are in [anchor-outcomes.csv](../runs/asm2-terminal/r9/anchor-outcomes.csv).

![Matched dose, imitation and training curves](../runs/asm2-terminal/r9/learning-and-native.png)

## Failure modes and contact geometry

R9 earlier failures, mutually exclusive primary native reasons:

| Transitions | Qualified | Forbidden airframe | Forbidden target | No signed touch | Footprint miss | Flight-volume exit | Flight speed limit | Contact-point speed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|98304|0|1|0|33|2|0|0|0|
|196608|2|21|0|6|0|7|0|0|
|294912|0|28|4|0|4|0|0|0|
|393216|0|13|0|12|4|6|0|1|

At294912 every earlier case touched, but none qualified. The28 forbidden-airframe cases comprise23 body-only and5 body/front first-touch manifolds; another4 front contacts hit forbidden target geometry. This is direct evidence that contact-reaching behavior can be retained without legal first-contact precision. All native cohorts and both policies' reason counts, including flight-speed and contact-point-speed failures separately, are in [native-rejection-counts.csv](../runs/asm2-terminal/r9/native-rejection-counts.csv). [Analysis JSON](../runs/asm2-terminal/r9/analysis.json) additionally records each forbidden body/target geometry combination and simultaneous-contact groups; raw secondary conditions remain in the unchanged scorer records.

The same nominal anchor illustrates the loss of qualified precision:

| R9 dose | First touch | Time s | Max recorded tangential point speed m/s |
|---:|---|---:|---:|
|98304|front007, outside footprint|.3460|.1879|
|196608|rear000, qualified|.3758|.0233|
|294912|body collision_0_570 + front007, forbidden|.3005|.3852|
|393216|body collision_0_570 + front007, forbidden|.3016|.3653|

These are signed first-contact manifolds and kinematic measurements, not claims of physically loaded forbidden contact or official force-area-time qualification. Native first-loaded records are preserved separately. No post-contact tail was added; G3.1.2 findings and their scope remain separate.

![Same earlier anchor at matched294912 dose](../runs/asm2-terminal/r9/matched-earlier-transient-anchor.png)

## Imitation versus autonomous behavior

| Dose | R8 B weighted raw NLL | R9 weighted raw NLL | R8 B tanh-action RMSE | R9 tanh-action RMSE |
|---:|---:|---:|---:|---:|
|0|−3.2794|−3.2794|.04086|.04086|
|98304|−2.6596|−2.6345|.07983|.08906|
|196608|−1.8973|−2.4591|.10863|.09785|
|294912|−2.1486|−2.6053|.09799|.08645|
|393216|−1.4913|−2.1726|.13537|.10769|

These are weighted in-source diagnostics on the fixed3439 pairs, not an independent prediction test. Continuous densities can have negative NLL. Final mean-action error is about20% lower than R8 B, but remains roughly2.64× BC's initial error. Per-source/category results are preserved in analysis.json; no fresh data enters this calculation.

The auxiliary-to-PPO gradient-norm ratio (ratio of the two recorded minibatch-mean norms per update) has median0.214%, range0.000761%–0.768%. This confirms a real, modest Actor signal; it does not measure gradient alignment or prove that a stronger coefficient would solve the task. Gaussian standard deviations were also recorded: likelihood improvement cannot alone establish better mean control, which is why deterministic action error is reported separately.

![Regularizer, gradient and exploration diagnostics](../runs/asm2-terminal/r9/regularizer-diagnostics.png)

All three demonstrated starts, from two original families, remain0/3 at every R9 checkpoint:

| Dose | Native demonstrated-start failures |
|---:|---|
|98304|3 no signed touch|
|196608|2 forbidden airframe,1 flight-volume exit|
|294912|3 forbidden airframe|
|393216|2 no signed touch,1 forbidden airframe|

At final, position RMSE against the fixed corresponding successful archived reference over their common elapsed interval is .136/.183/.216m; attitude RMSE8.9°/12.3°/11.7°; rotor-thrust RMSE .028/.034/.019N for stationary_0-minus, stationary_0-plus, stationary_2-plus respectively. Full velocity, rates, held targets and command deviations are in analysis.json. This is a descriptive comparison with one reference, not a mandatory trajectory or a new task criterion.

![Autonomous demonstrated-start trajectories and archived controls](../runs/asm2-terminal/r9/demonstrated-recentering-3d.png)

R9 records7 training earlier successes in843 completed earlier-band episodes, versus R8 B's3 training successes. These correlated stochastic-training events do not replace independent native outcomes. The transient native anchor and final failure are reported without selecting a training-only success story.

## Frozen fresh evaluation

Selected equals final, so one94-case suite serves both roles; no duplicate evaluation or replacement cases were added.

| Fresh cohort | Archived R8 B final | R9 final |
|---|---:|---:|
|Earlier perturbations|0/36|0/36|
|Independent-source rotation|1/18|0/18|
|Independent-source braking|11/18|12/18|
|Independent-source terminal|18/18|18/18|
|Reached handovers|0/4|0/4|

Earlier fresh failures:12 no signed touch,11 forbidden airframe,6 footprint misses,6 flight-volume exits,1 contact-point-speed rejection. Rotation:10 flight-volume exits,5 forbidden airframe,2 no signed touch,1 contact-point-speed rejection. Braking:12 qualified,5 forbidden airframe,1 contact-point-speed rejection. All reached handovers exit the flight volume. Terminal18/18 qualifies.

Fresh earlier cases are perturbations of development-source anchors; the other cohorts use separate original sources. This is the frozen, historically evaluated R8 fresh suite, not globally unseen blind data. The inherited console initially groups phase labels into54 rotations; the authoritative saved cohort report separates earlier36 and independent rotation18, as above.

## Accounting, reproducibility and artifacts

- 393216 completed and charged transitions,384 finite full rollout archives,24576 regularized Actor minibatches, one charged restart, zero abandoned updates. Learning wall time5251.8s, excluding evaluation pauses.
- 726 charged/completed native rollouts:552 development +12 demonstrated starts +94 fresh +68 replay reproductions. The unused105 allowance expires; it is not an extension budget.
- 68 replays match their original qualification, reason, contact and time, with bitwise-identical complete physics arrays. All726 cases retain initial complete physical/actuator state,0.1ms state/rotor/held-target history, CTBR/allocator outputs, contact records and viewer replay.
- [Offline verification](../runs/asm2-terminal/r9/verification.json) checks6,479,728 physics intervals and129,859 exact autonomous Actor commands; maximum motor-filter residual5.55e−17N and allocator residual1.11e−16N. [Checkpoint verification](../runs/asm2-terminal/r9/checkpoint-verification.json) checks all five dose checkpoints, portable exports, exact Adam step increments and schedule metadata. An initial verification ran before the final rollout archive was published on normal trainer shutdown; after shutdown all384 were present and passed, without rerunning training.
- Original preflight pins, R8 evidence, Phase4 and Fixed A remain intact. The two explicitly amended orchestration hashes are tracked separately; no frozen physical/scoring/learning source was changed.

Representative existing-format 3D replays: [transient qualified earlier nominal](../runs/asm2-terminal/r9/native-records/0259/replay.npz), [same anchor final body/front rejection](../runs/asm2-terminal/r9/native-records/0535/replay.npz), [final demonstrated stationary_2 forbidden touch](../runs/asm2-terminal/r9/native-records/0606/replay.npz). Each directory also contains `physics.npz`, `contacts.json`, `started.json` and `outcome.json`. Analysis/verification scripts are preserved in the R9 run directory and perform saved-data processing only; existing outputs are write-once.

## Decision and one next action

**No-Go for scientific adoption and promotion.** No multi-anchor earlier gain, no robust earlier anchor, no final persistence, no fresh earlier gain, and no checkpoint passes the original braking retention gate. This does not establish global unreachability or reject every possible demonstration-regularization method; it rejects this preregistered candidate under one seed and the frozen task.

The strongest supported bottleneck is converting demonstration-related behavior into closed-loop qualified first-contact precision across starts. BC never possessed a qualified demonstrated-start skill, and R9's better late imitation retained more incorrect contact without resolving that gap. Narrow data coverage, off-demonstration state drift and the small auxiliary signal are plausible contributors, not separately proven causes. No conclusion about actuator-memory insufficiency or a need to change geometry follows.

**Exactly one next experiment, NOT LAUNCHED:** a bounded corrective-demonstration pilot at the existing three demonstrated starts, adding dynamics-valid action labels at states actually visited by the learned policy. Test whether a refitted autonomous policy can achieve native-qualified first contact while retaining braking before allocating another PPO campaign. Keep physics/scorer/45-D input unchanged; expert commands are training labels only, never an evaluation-time controller. This targets the observed precision/distribution gap rather than rerunning R8 or increasing λ without evidence. A separate manifest and approval are required.
