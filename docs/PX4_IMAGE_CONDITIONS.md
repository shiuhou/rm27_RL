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
