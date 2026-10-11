# Complete-mission condition development

`experiments.image_conditions` adds declared disturbances before the normal
image detector, native VIO or PX4 sensor input. Frame loss returns no frame;
latency queues actual rendered images and retains their acquisition time.
Brightness/noise alter actual pixels. Flow loss changes the sensor quality;
range jumps change the measured range; heading faults rotate magnetometer
measurements; low thrust attenuates actual motor commands. Recorded commands
retain the realized fractions, separately from the requested fractions.
An explicitly configured force enters the physical plant, with its actual
vector saved in each row. These are simulation assumptions.

## Observed timestamps and delayed camera

The first100 ms delay development fails before flight. The native player's
single shared monotonic clock incorrectly rejects an old camera sample that
arrives after a fresh IMU. The failure is retained in
`px4-image-delay-development-01`. A separate exact publisher copy now checks
acquisition order independently for camera and IMU; its path/hash and helper
are recorded. This development copy leaves the active frozen nominal
publisher unchanged. Backwards/repeated acquisition within either sensor
still fails. No delayed image is restamped.

`px4-image-delay-development-02` completes one full image/VIO mission with
100 ms primary-camera latency, correct acquisition clocks,6 ms contact
inference and1.685 mm independent resting XY. Native camera and IMU still
come from the current flight. This is development evidence; independent
surface clearance, measured EKF2 fusion and reserved variation acceptance
are additional gates.

## Other development failures

`px4-image-composite-development-01` combines fixed brightness, per-frame
pixel noise,1.2× IMU noise, a lateral force and a range jump. It fails at
HOVER: native VIO has only3 TRACKING publications and149 LOST publications;
unstable initial velocity is accepted and the EKF estimate exceeds the flight
envelope. This run is retained. It does not establish noise robustness or safe
physical termination. The reduced diagnostic V2 removes stochastic noise
and applies the force only during ROUTE to separate initialization from later
disturbance handling. Further fixes require fresh development and acceptance.

The reduced V2 composite now completes one physical mission in
`px4-image-composite-development-02`: fixed primary/downward brightness 0.85/0.9,
0.003 N lateral force during ROUTE and a 0.05 m range jump at HOVER. Contact
inference is 4 ms, there is no second impact, and resting XY is 2.892 mm.
Separate fusion/surface/corridor membership checks remain additional evidence.

The delayed and reduced-composite flights now pass their separate recorded
local surface, native recovery-corridor membership and actual EKF2 EV fusion
checks. Both remain development flights.

Noise also exposes a host detector cost: the original bearing detector builds
a full image mask for each small component. An equivalent implementation
uses component bounding statistics and one selected region. Randomized output
comparisons retain the original ambiguity, confidence and freshness results.
A noisy actual-scene frame benchmark retains exactly equal outputs and
measures 0.3911 s versus 0.00221 s (177×) in
`px4-noisy-bearing-compatibility-01`; this one-frame host measurement does not
qualify M3C real-time performance. It is an explicit experiment option.

`vio_initial_admission` adds a development first-registration guard against
fresh PX4 flow navigation: horizontal velocity disagreement must be at most
0.2 m/s and the largest initial position covariance eigenvalue at most 0.01 m².
It never aligns to truth or realigns continuously. The failed composite's
first native horizontal velocity is about 0.9 m/s and position covariance
0.161 m², while accepted baseline initial covariance is about 0.0025 m².
Noisy guard flights remain separate development and cannot be counted as a
VIO or noise qualification merely because an outlier was rejected.

Both initial-admission noisy flights subsequently fail when native OpenVINS
exits on negative covariance. Isolated numerical diagnosis and continued
takeoff-footprint LAND development are recorded in
[native numerical report](PX4_NATIVE_NUMERICS.md). The original installation
remains frozen for reserved batches.

The separate sensor-only permanent Offboard-loss development passes, followed
by five reserved independent MTF launches in `px4-offboard-loss-batch-01`:
**5/5 physical terminations,0/5 successful tasks**. Upstream setpoints stop;
the configured onboard PX4 Offboard failsafe must LAND without an injected
external landing command. The same process continues until actual landed
state, automatic disarm, stopped motors and bounded physical rest. Maximum
resting XY is5.048 mm. This covers dock hover command loss; full-route
communication loss and invalid-navigation termination remain separate.

## Batch gate

`experiments.image_condition_batch` freezes all declared inputs, launches
independent instances and scores each attempt in its own process. Successful
tasks require independent contact/recovery/hover/rest, assigned tag, actual
EKF2 EV fusion, no unexpected failsafe, recorded local surfaces, measured
IMU observer rate and every native recovery step inside the continuously
certified corridor. A safe termination is counted separately and must pass
its physical termination gate. A50-case gate requires at least48 successful
tasks and verified handling of every remaining attempt. No50-case gate has
passed yet. Each completed recording is archived losslessly after scoring.

The50-case V2 batch is running in `px4-image-conditions-batch-01`. Four
early unhandled failures prevent a passing result: attempt04 has unstable
dock contact during TAKEOFF;09 rejects disarmed rest;15 rejects final LAND
alignment;18 also rejects final LAND alignment. All remaining cases continue with unchanged inputs and all failed
attempts remain in the batch. Complete-route downward faults separately run
in `px4-full-return-fault-batch-01`; its wrong-ID attempt09 rejects final
alignment after successfully reacquiring the assigned tag.

## Fresh first-registration and landing development

`tools.px4.run_initial_admission` runs an explicit saved flight-runner copy,
leaving the active reserved implementations unchanged. Initial registration
adds a maximum0.15 m/s vertical disagreement and five consistent observations
over0.2 s, with gaps at most0.15 s. Horizontal and covariance admission remain
unchanged. This is one initial rigid registration, never continuous alignment
to physics. New delayed-camera seeds424001/424002 both pass the complete
physical task and separate actual EV fusion, local surface and native recovery
membership audits (`px4-stable-vio-admission-development-score-01`/`-02`).

The post-LAND development uses fresh PX4 landed state, heartbeat, measured
range and stopped actuator outputs. Estimated velocity below the aiding floor
does not veto established landed/disarmed state. Physical speed at most0.01 m/s
and actual motor force below1e-5 N remain mandatory independent scoring gates.
There is no velocity reset or physical force input to mission completion.
`px4-ground-rest-bias-development-01` deliberately adds0.03 m/s to estimated
vertical velocity in POST only, retaining the original telemetry. It passes
the full task with2.580 mm independent resting XY and reported estimated
vertical velocity0.02968 m/s. Its separate audit
`px4-ground-rest-bias-development-score-01` passes every required check.

The explicit landing runner can also return to ALIGN/REACQUIRE when fresh
navigation or the assigned image shifts outside the original3 mm admission
at final LAND commitment. The same two-attempt bound is retained. Invalid
navigation still rejects. Delayed-camera/wrong-ID development seed424005
reacquires and completes with3.514 mm resting XY and a passing independent
audit (`px4-landing-retry-development-score-01`); it does not enter the new
final-alignment retry branch. A separate declared real-pixel translation
experiment exercises that branch before any acceptance claim.

None of these development results changes the failed V2 batch or establishes
a replacement reserved gate. Seeds from failed reserved sets are excluded
from the development runs above.

## Causal actor and reserved replacement gates

Review of the original V3 runner found physical contact/tilt checks that could
abort the actor, actual motor force in completion, and simulator initial yaw.
Its retained20/20 physical tasks therefore do not qualify strict actor isolation.
`tools.px4.causal_flight` makes an explicit saved copy: surveyed initial yaw,
fresh PX4 attitude/navigation and images control the actor. Physical contact,
force and parked-aircraft checks only reject the result independently after
actor completion. Sensors and independent scoring retain the physical state.
The original frozen implementations remain unchanged during their batches.

`px4-causal-image-mission-development-score-01` passes all independent gates,
with1.844 mm resting XY. The real-pixel final admission pulse now actually
enters `landing_alignment_retry`; its causal flight and separate score pass
in `px4-causal-landing-pulse-development-01` and `-score-01`, with1.757 mm
resting XY. The original3 mm final admission remains unchanged.

V5 declares a3 s continuous absent-return-image budget while retaining the
15 s bound for a fresh-image reacquisition climb. This engineering budget
limits drift during permanent image loss. New seed424010 passes physical
termination without unintended dock contact,6.955 mm resting XY and14.344 s
continued physics (`px4-causal-permanent-tag-development-score-01`). It is
a failed task. Fresh images and source times are never synthesized.

Physical uncertainty is explicitly applied before simulation in
`tools.px4.physical_fixture`: initial XY at most4 mm, initial yaw at most3 degrees,
and0.9–1.1 factors for motor lag, maximum thrust and contact time. Geometry and
150 g mass remain fixed. Development424012 combines[2,-2] mm initial XY,+2 degree
yaw,1.1 lag,0.95 thrust,0.9 contact time and50 ms primary delay. Its task and
independent score pass,2.733 mm rest. Exact fixture replay has zero error over
41,834 frames. Native1 ms versus0.5 ms contact convergence passes (impulse
2.4632%, peak4.4895%). The retained original convergence report counts
zero-force contact candidates separately from loaded impacts; every negative
or grazing nonzero load still rejects front-only classification.

Fresh sets are preregistered before launch:20 nominal seeds466001–466020,
60 condition seeds467001–467060,15 downward-fault seeds468001–468015, and
five command-loss seeds469001–469005. The60-case set extends the original
50 condition ranges with five initial-pose and five joint physical/image
variants; it requires58 tasks and verified handling of every remaining case.
`tools.px4.stable_condition_batch` freezes sources, binary, parameters, data
and certificate. No replacement reserved gate is claimed passed before its
actual complete summary. All failed original/development attempts remain.
