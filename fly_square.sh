#!/usr/bin/env bash
# Take off, fly a 10 m square at 3 m, land: a smoke test of the stack (SIM ONLY: arming checks off).
#   ./fly_square.sh [LAPS] [--vio]      LAPS default 1; --vio switches the EKF to the VIO after takeoff

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/fly_common.sh"

LAPS=1; USE_VIO=0; Z=3.0
for a in "$@"; do
  case "$a" in
    --vio) USE_VIO=1 ;;
    ''|*[!0-9]*) echo "unknown argument: $a"; exit 1 ;;
    *) LAPS=$a ;;
  esac
done

leg() {   # leg <x> <y>: fly to the point (8 s at 20 Hz)
  ros2 topic pub -r 20 -t 160 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: $Z}, orientation: {w: 1.0}}}" >/dev/null
}

say "waiting for MAVROS"
wait_fcu

say "speed 2 m/s, arming checks off (sim)"
set_retry f WPNAV_SPEED 200 || exit 1
set_retry i ARMING_CHECK 0 || exit 1

takeoff_hover "$Z" || exit 1
if [ "$USE_VIO" = 1 ]; then switch_to_vio || exit 1; fi

for ((i=1; i<=LAPS; i++)); do
  say "lap $i of $LAPS"
  leg 10 0; leg 10 10; leg 0 10; leg 0 0
done

if [ "$USE_VIO" = 1 ]; then say "switching back to GPS"; src 1; fi
say "landing"
mode LAND
wait_alt le 0.2 60 || say "did not confirm touchdown, continuing anyway"
sleep 3
stop_recorder