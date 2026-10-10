# M3C / PX4 interface inventory — 2026-10-10

## Result

**Read-only inventory VERIFIED; spare physical link still PARTIAL.** Live reads
were completed approximately16:49–16:52 Asia/Shanghai, inside the approved10-minute
bound. No data UART opened, no stty/pinmux/configuration/parameter/firmware/wiring
change, no capture/DDS/heartbeat sender or Stage A/B repeat. Existing management
connections only: M3C SSH and PX4 COM19 USB MAVLink console. All sessions ended.

The important new restriction is **USB fallback is not plug-and-play on this
M3C image**: the active controller is a USB device providing the SSH management
network, and the running kernel has `# CONFIG_USB_ACM is not set`. The enabled
`CONFIG_USB_CONFIGFS_ACM=y` is gadget-side functionality, not the host CDC-ACM
driver needed to expose PX4 as ttyACM on M3C. No kernel/module/role change attempted.

Preferred next direction remains a separate UART, subject to physical pin routing
confirmation. Do not treat unclaimed kernel pin ownership or a /dev node as proof
of a free, correctly muxed, voltage-compatible external UART.

## PX4 fresh evidence

`px4-inventory.txt`, command helper exit0:

- MICOAIR_H743_V2, v1.17.0, exact hash
  d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
- Before/after commander Disarmed; EKF2_EV_CTRL read0 twice. Observed FC heartbeat
  base_mode29. These are bounded observations, not an exhaustive safety history.
- Current selected gyro6684690 / accel6946834, instance0 (BMI088), unchanged from
  prior qualified identity readback. Historical BMI270 is not substituted.
- uxrce_dds_client exists but is not running.

| PX4 device | Fresh observation | Decision |
|---|---|---|
| ttyS0 | MAVLink instance0,57600 | Already assigned; not a spare |
| ttyS2 | MAVLink instance1,115200,version2,Onboard,5760 B/s budget | Preserve M3C link |
| ttyS3 | MAVLink instance2,115200,comp88,msg106/132 ~48 Hz | Preserve optical-flow/ToF |
| ttyS5 | wq:ttyS5 visible; target source maps RC | Reserve; do not repurpose |
| ttyS1/4/6/7 | Device nodes exist; no explicit owner shown in ps | Candidates ONLY; ps is not a full FD audit or proof of external pads |
| ttyACM0 | MAVLink instance3 / USB console currently owns it | DDS cannot concurrently take over this same stream |

ttyS2 RX0 is expected with the previous sender stopped. Its historical comp191
counters and MAVLink2 state persist; no new Stage A/B parsing PASS is claimed.
Loss counters are cumulative across prior sessions and sequence restarts, not
packet-loss measurements from this inventory. USB ttyACM0 reports2000000 as a
virtual-port setting; it is not evidence for any physical UART baud.

Only the eight allowlisted read commands in px4_inventory.py were sent, with
fresh disarmed heartbeat guards. USB shell release is in finally. No parameter
write, stream-rate command, start/stop, reboot or logger operation was sent.

## M3C fresh evidence

`m3c-inventory.json` is a read-only proc/sysfs/device-tree snapshot, collected by
piping m3c_inventory.py to `ssh ... python3 -B -`; no remote script/file created.
The helper never opens /dev. It did not import Maix, change pinmux, mount debugfs,
load modules or install dependencies.

Runtime kernel4.19.125/aarch64, device-tree model MaixCAM2, axera UART driver.
That is a software image identity, not a claim the user's M3C carrier name is wrong.

| M3C tty | Controller address | Ownership observed | Qualification |
|---|---|---|---|
| ttyS0 |0x04880000|No matching userspace FD in snapshot|Candidate only; boot/ROM use and carrier access unknown|
| ttyS1 |0x04881000|No matching userspace FD in snapshot|Preferred spare candidate to verify physically|
| ttyS2 |0x04882000|No matching userspace FD; prior sender/receiver stopped|Existing wired PX4 link; reserve|
| ttyS3 |0x06080000|No matching userspace FD|Candidate only; pin alternatives may conflict|
| ttyS4 |0x06081000|launcher PID65800 FD4 opens /dev/ttyS4|Occupied; do not stop or reuse|
| ttyS5 |0x06082000|No userspace FD, but pinctrl explicitly assigns UART5 CTS/RTS/TX/RX|Reserved pending ownership/pin-use investigation|

All six UART DT nodes status=okay. Lack of a userspace FD does not exclude kernel,
boot firmware, MMIO or future application use. /proc/consoles shows tty0 and
pstore-1, no current ttyS console; this does not rule out boot-time serial use.

Pinctrl UART0/1/2/3 pins report UNCLAIMED, including the already working UART2.
Therefore this field demonstrably cannot certify physical mux or free pins here.
Most UART nodes have no pinctrl-0 property, while UART5 has explicit assignments.
No /dev/mem or direct register manipulation/read was attempted.

UART2 driver counters were the same in two reads: tx188087,rx1266717,fe68,brk7.
These are cumulative counters. Their errors are not newly induced by this read-only
inventory and cannot be assigned to the earlier startup anomaly from these reads.
No active receive test occurred; unchanged counters do not certify line quality.

### USB host feasibility

Evidence: m3c-interface-followup.txt and m3c-usb-kernel-retry.txt.

- Controller8000000.dwc3: role=device, UDC state=configured, current_speed=high-speed.
- DT dr_mode=otg, maximum-speed=high-speed; kernel DWC3_DUAL_ROLE and XHCI enabled.
  These establish software capability components, not a working host connection.
- Gadget g1 binds this controller, with ncm.usb1 and rndis.usb0 active.
- SSH address10.18.198.1 is on usb1; both usb0/usb1 resolve to gadget devices.
  **Inference:** switching this controller to host would disrupt the present SSH
  management path. No alternative active management route is established here.
- CONFIG_USB_ACM is not set; no registered cdc_acm driver or ttyACM/ttyUSB device
  on M3C. /lib/modules/4.19.125 does not exist. No simple modprobe fix is assumed.
- Gadget CONFIGFS_ACM=y is not a substitute for the host ACM driver.

Thus USB fallback needs at least a separately approved compatible host-driver
solution, safe host/VBUS wiring and a management-access plan; PX4 USB is also
currently occupied by MAVLink console. Do not change USB role or take over either
USB endpoint to test this automatically.

## Correlation with existing schematic

Read-only visual check of the already rendered M3C_378C V1.0 PinOut page12
(`tmp/pdfs/m3c-pinout-12.png`; original supplied SCH_M3C_378C.pdf):
UART1 RX GPIO0_A31 / J1pin6, TX GPIO0_A30 / J1pin8 are labelled3.3 V;
UART2 RX GPIO1_A1 / J1pin2 and TX GPIO1_A0 / J1pin4 are labelled3.3 V.
These are module connector positions, **not a verified carrier solder-pad map**.
UART4 alternatives include1.8 V pins, reinforcing why tty enumeration is not a
wiring guide. No continuity/voltage or signal measurement was performed.

Candidate to confirm, not a wiring instruction: M3C UART1 plus a physically
accessible unused PX4 UART from ttyS1/4/6/7. No final port selection can be made
from ps and a module schematic alone. Existing UART2 and optical flow stay intact.

## Scope, verification and preserved failures

Working tree main b8e7299cf090f5150f9f262ebd3b232a824cc526; unrelated dirty files
preserved. New inventory evidence is isolated here. Root HANDOFF/VAULT_UPDATE and
new_plan receive additive pointers; frozen prior reports are not edited.

Both diagnostic scripts parsed successfully; offline allowlist check accepted8
read commands and rejected5 representative mutations without constructing a Link
(`offline-check.txt`). This is a limited guard check, not a production qualification.
No VIO regressions rerun because no VIO/adapter/contract code changed.

M3C collection and PX4 helper exited0. First USB-kernel follow-up failed in remote
shell parsing because Windows/SSH quoting removed regex quotes; no command in that
failed string ran. Retained m3c-usb-kernel.txt; retry used simple grep -e terms and
succeeded. Missing module-directory and absent optional DT properties remain in
evidence, not silently treated as a successful read.

Prior design manifest33 entries and transport manifest48 entries verify with0
mismatches. Their hashes remain respectively
eefd951fa8c6df506dfccb6a2eaeb2a8f98e668e023d9db0dcf68b9cc3a01a59 and
54397ebcd7a0f7b06805f10241ceb23693fae47adef24c82901d2f1f2c776aa7.
Stage A/B files were not modified or rerun. No Vault changes or push.

## Next boundary

Inventory permission is consumed; do not continue with settings or wiring changes.
**One next action:** obtain the hardware designer's carrier-pad mapping confirming
whether M3C UART1 TX/RX/GND and one unused PX4 UART are actually accessible and
voltage-compatible, without changing the current wiring. This is a request for
hardware evidence, not authorization to connect new pins. Only then propose the
specific link and separately approve baud/firmware/topic-export work.

VIO-S0 remains PARTIAL; dedicated-link availability PARTIAL; USB plug-in fallback
BLOCKED on driver/role/management prerequisites; high-rate transport, clock mapping
and OpenVINS source-model qualification remain UNKNOWN. Stage C NOT STARTED.
