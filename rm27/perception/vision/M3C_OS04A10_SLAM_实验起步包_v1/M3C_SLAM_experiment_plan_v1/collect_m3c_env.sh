#!/bin/sh
# Read system metadata; only writes reports under the requested output directory.
# Does not install packages, open the camera, change parameters, or contact a network.
# This is an environment collector, not a SLAM/camera performance test.
set -u
OUT=${1:-./m3c_env_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT" || exit 1
REPORT="$OUT/environment.txt"
if [ -e "$REPORT" ]; then
    printf '%s\n' "Refusing to overwrite $REPORT; choose a new output directory." >&2
    exit 2
fi
run() {
    printf '\n===== %s =====\n' "$1"
    shift
    "$@" 2>&1 || printf '[unavailable / failed: exit %s]\n' "$?"
}
{
    run 'UTC date (check whether board clock is set)' date -u
    run 'Kernel' uname -a
    run 'Userspace word size; not a unique SoC identifier' getconf LONG_BIT
    run 'OS release' cat /etc/os-release
    run 'Device tree model' sh -c 'if [ -r /proc/device-tree/model ]; then tr "\000" "\n" < /proc/device-tree/model; else echo unavailable; fi'
    run 'Device tree compatible' sh -c 'if [ -r /proc/device-tree/compatible ]; then tr "\000" "\n" < /proc/device-tree/compatible; else echo unavailable; fi'
    run 'CPU information' cat /proc/cpuinfo
    run 'CPU summary' lscpu
    run 'Kernel boot arguments; may describe reserved memory' cat /proc/cmdline
    run 'Linux-visible memory and CMA' sh -c 'grep -E "^(MemTotal|MemFree|MemAvailable|Buffers|Cached|SwapTotal|SwapFree|CmaTotal|CmaFree):" /proc/meminfo'
    run 'Memory overview' free -h
    run 'Active swap devices (no changes made)' cat /proc/swaps
    run 'Block devices' sh -c 'lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINTS 2>/dev/null || lsblk'
    run 'Filesystem space' df -h
    run 'Shell ELF architecture' sh -c 'file /bin/sh; p=$(readlink -f /bin/sh 2>/dev/null); if [ -n "$p" ]; then file "$p"; fi'
    run 'C compiler target' sh -c 'command -v gcc >/dev/null 2>&1 && gcc -dumpmachine'
    run 'C compiler version' sh -c 'command -v gcc >/dev/null 2>&1 && gcc --version | head -n 2'
    run 'CMake version' sh -c 'command -v cmake >/dev/null 2>&1 && cmake --version | head -n 1'
    run 'OpenCV pkg-config versions' sh -c 'if command -v pkg-config >/dev/null 2>&1; then pkg-config --modversion opencv4 2>/dev/null || pkg-config --modversion opencv 2>/dev/null; else echo unavailable; fi'
    run 'Python version' sh -c 'command -v python3 >/dev/null 2>&1 && python3 --version'
    run 'Video device names; existence does not prove V4L2 capture works' sh -c 'ls -l /dev/video* /dev/media* 2>/dev/null || true'
    run 'Thermal readings (units determined by the platform)' sh -c 'for d in /sys/class/thermal/thermal_zone*; do [ -d "$d" ] || continue; echo "$d"; for f in type temp; do [ ! -r "$d/$f" ] || { printf "%s=" "$f"; cat "$d/$f"; }; done; done'
    run 'CPU frequency readings (units determined by the platform)' sh -c 'for f in /sys/devices/system/cpu/cpufreq/policy*/scaling_cur_freq /sys/devices/system/cpu/cpufreq/policy*/scaling_max_freq; do [ ! -r "$f" ] || { printf "%s=" "$f"; cat "$f"; }; done'
} > "$REPORT"
cat > "$OUT/MANUAL_NOTES.md" <<'NOTES'
# Fill in manually; do not infer these from the product family

- Physical board/SKU/revision:
- Product specification document and its revision:
- SoC identity (cross-check device tree, supplier documentation, SDK):
- Carrier board revision and schematic:
- Power source and rated interface voltage:
- Camera module manufacturer/revision and connector pinout:
- Lens model, current focus setting and mounting direction:
- Image/firmware filename and checksum:
- MaixPy / MaixCDK / AX SDK versions or Git commits:
- Exact existing camera startup command and configuration:
- Available image width, height, sensor mode and frame rate:
- Meaning of the driver timestamp (exposure / SOF / buffer arrival / unknown):
- Whether an IMU is physically installed and which one:
- All attached peripherals and their power consumption:

This collector does NOT prove camera support, synchronization, or SLAM speed.
Device-tree names can be generic. CPU architecture alone is not proof of a SoC SKU.
Review the report for device identifiers before sharing outside the team.
NOTES
printf 'Created %s and %s\n' "$REPORT" "$OUT/MANUAL_NOTES.md"
