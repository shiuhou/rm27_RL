# M3C MAVLink2 / ODOMETRY bench validation — 2026-10-10

## Verdict and boundary

VERIFIED: UART TX, MAVLink2, HEARTBEAT parsing, ODOMETRY parsing and
vehicle_visual_odometry publication, including running/stopped/restarted attribution.
This is synthetic transport evidence, NOT VIO, clock synchronization, EKF fusion,
loss-free transport, endurance, calibration or flight qualification. Prior VIO-P
and VIO-S0 gates are unchanged. No PX4 parameter writes, arm, setpoints, firmware,
pinmux, camera or production estimator changes were performed.

Final readback (`px4-final-readback.txt`): commander Disarmed, EKF2_EV_CTRL=0,
UART instance #1 RX=0 after sender termination, MAVLink version2 retained.
Last topic timestamp6970373496 was83.923523s old. Both Stage B runs ended normally
and reported termios restoration equal=true; no venv Python sender remained.

## Artifacts and setup

M3C: /dev/ttyS2,115200,8N1,no flow control,sysid1/compid191.
PX4: MICOAIR_H743_V2,v1.17.0,d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
The PX4 UART endpoint is also /dev/ttyS2; COM19 is USB inspection only.
User previously confirmed all propellers removed. Fresh Disarmed/EV_CTRL readbacks
preceded synthetic transmission. Sender also requires fresh disarmed PX4 heartbeat
and aborts on armed/stale heartbeat; it does NOT independently query EV_CTRL.

Modified existing ../m3c-ping-20261009/heartbeat_only.py minimally with optional
--mavlink2 and bounded duration, retaining v1 default and UART guards/restoration.
New independent mavlink_odometry_test.py leaves heartbeat script intact.
Deployed at /root/rm27-mavlink-test-20261010/ on M3C. Isolated venv Python3.13.2,
pymavlink2.4.49,lxml6.1.3,fastcrc0.5.0. DNS prevented direct pip installation;
host-downloaded CP313/aarch64 wheels were installed offline in that venv only.
No system Python changes. api-final.txt preserves installed odometry_send signature.

## Stage A — PASS before Stage B

MAVLINK20=1 precedes pymavlink import; explicit v20 common dialect, wire magicFD.
Heartbeat is ONBOARD_CONTROLLER/AUTOPILOT_INVALID at1Hz. Console prints port,
baud, versions, sysid/compid, count and actual frame hex (21bytes).

`px4-stage-a-running.txt`: instance#1 RX21.0B/s,sys1/comp191,msg0 Rate1.0Hz,
MAVLink version2,serial /dev/ttyS2@115200. MAV_PROTO_VER was read as1; it was not
changed. Actual negotiated link version, not an assumed parameter mapping, is proof.

`stage-a-blUswkNE.log`:108 writes, intentionally SIGTERM-stopped after gate PASS
to free UART; finally termios_equal=true. KeyboardInterrupt traceback is from that
intentional signal. Do not claim a completed180-second dwell.
Earlier `stage-a-7B2cmRrJ.log` has a NUL-filled tail and no successful completion;
preserved as an incomplete attempt, cause UNKNOWN, excluded from completion proof.

M3C command (standalone, do not run concurrently with Stage B):
```sh
/root/rm27-mavlink-test-20261010/venv/bin/python /root/rm27-mavlink-test-20261010/heartbeat_only.py --mavlink2 --seconds 180
```

## Stage B — PASS

Exact firmware source inspected before implementation:
https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_receiver.cpp
Local source copy mavlink_receiver.cpp is a text representation, not a pristine blob.
handle_message_odometry accepts LOCAL_NED position and BODY_FRD velocity;
VISION routes to _visual_odometry_pub / vehicle_visual_odometry. Use installed
enum constants: MAV_FRAME_LOCAL_NED(1),MAV_FRAME_BODY_FRD(12),
MAV_ESTIMATOR_TYPE_VISION(2),quality100,reset_counter0.

Pose(1,2,-0.5)m,q[1,0,0,0],zero linear/angular velocities. HEARTBEAT1Hz,
ODOMETRY10Hz. Covariance upper triangle21 entries, diagonal indices0/6/11=.01,
15/18/20=.0025,off-diagonal0. These are assumed independent test uncertainties
(0.1m,0.1m/s,0.05rad,0.05rad/s standard deviations), not measured confidence.
Timestamp uses M3C monotonic microseconds. No TIMESYNC sent; PX4 sync_stamp
conversion and small timestamp_sample-to-timestamp differences do NOT establish
clock synchronization or sample age accuracy.

Only after fresh EV_CTRL=0, Disarmed and propeller-removal confirmation:
```sh
/root/rm27-mavlink-test-20261010/venv/bin/python /root/rm27-mavlink-test-20261010/mavlink_odometry_test.py --seconds 45 --bench-ev-disabled-confirmed
```
The confirmation flag is an operator assertion, not a parameter read or write.

| Observation | Evidence | Result |
|---|---|---|
| Before | px4-stage-b-before.txt | topic never published |
| Running | px4-stage-b-running1.txt | RX2470.3B/s,msg33110.0Hz,msg0~0.9Hz,version2 |
| Stop | px4-stage-b-stopped.txt | RX0; timestamp6909383485 unchanged across ~7s |
| Restart | px4-stage-b-restarted.txt | RX2466.1B/s,msg33110.0Hz; timestamps advance again |
| Final stop | px4-final-readback.txt | RX0,old retained topic,Disarmed,EV_CTRL0 |

First five running timestamps6881272839..6881666701us: four intervals average
98.466ms (~10.16Hz over this small window). Restart five6944663802..6945069615us:
average101.453ms (~9.86Hz). Both PX4 MAVLink rate displays10.0Hz. All observed
samples have requested pose,q,zero velocities,pose_frame1,velocity_frame3,
position/velocity variance[.01,.01,.01],orientation variance[.0025,.0025,.0025],quality100.

stage-b-run1.log:45.01041548s,45HEARTBEAT,451ODOMETRY,restored=true.
stage-b-restart-lRPXifKK.log:30.00289968s,30HEARTBEAT,301ODOMETRY,restored=true.
Counts include initial/boundary ticks. First run stopped automatically before an
attempted SIGTERM (no such process); do not describe that as a successful kill.
Independent runs reset sequence numbers; cumulative PX4 lost counters147/163
cannot be interpreted as this test's physical link loss fraction. Loss-free NOT proven.

## Tests and preserved scope

Fresh M3C `venv/bin/python -m unittest -v test_heartbeat test_odometry`:
4 tests PASS (`tests-final.txt`): v1/v2 framing+CRC+fields,ODOMETRY fields/covariance,
disarmed/fresh-heartbeat guard. These are code tests, separate from real PX4 evidence.
No commit/push, Vault ingestion or old evidence rewrite. No further transmission
is needed for this requested minimal slice. Clock alignment, measured estimator
covariance and real VIO data remain future separate work, not silently enabled.
