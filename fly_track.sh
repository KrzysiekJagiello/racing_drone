#!/usr/bin/env bash
# Take off, fly a trajectory CSV, land (SIM ONLY: arming checks off, guided limits raised).
#   ./fly_track.sh [TRAJ.csv] [--vio]
# Without --vio the CSV is in the world frame (EKF on GPS), with --vio in the start frame (EKF on VIO).
# No CSV given: the 3-lap examples in race_track_tools/example_trajectories/.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/fly_common.sh"

USE_VIO=0; TRAJ=""
for a in "$@"; do
  case "$a" in
    --vio) USE_VIO=1 ;;
    *.csv) TRAJ="$a" ;;
    *) echo "unknown argument: $a"; exit 1 ;;
  esac
done
if [ -z "$TRAJ" ]; then
  if [ "$USE_VIO" = 1 ]; then TRAJ="$HERE/race_track_tools/traj_start_frame.csv"; else TRAJ="$HERE/race_track_tools/traj_world.csv"; fi
fi
if [ ! -f "$TRAJ" ]; then
  echo "trajectory file not found: $TRAJ"
  exit 1
fi
if [ "$USE_VIO" = 1 ]; then say "frame: START frame (EKF on VIO)   file: $TRAJ"; else say "frame: WORLD (EKF on GPS)   file: $TRAJ"; fi

Z0=$(awk -F, 'NR==2{print $4}' "$TRAJ")
[ -n "$Z0" ] || { echo "cannot read the start height from $TRAJ"; exit 1; }

say "waiting for MAVROS"
wait_fcu
wait_gps || exit 1

say "raising guided limits, arming checks off (sim)"
set_retry f WPNAV_SPEED    900 || exit 1     # cm/s, ceiling: the trajectory sets the speed
set_retry f WPNAV_ACCEL    900 || exit 1     # cm/s^2, ceiling
set_retry f WPNAV_SPEED_UP 500 || exit 1
set_retry f WPNAV_SPEED_DN 500 || exit 1
set_retry f WPNAV_ACCEL_Z  700 || exit 1
set_retry f ANGLE_MAX      4500 || exit 1    # centi-deg, default 3000 caps acceleration at ~5.7 m/s^2
set_retry f PSC_JERK_XY    20 3 || say "PSC_JERK_XY not accepted: the path will be followed late"   # default 5 m/s^3
set_retry f PSC_JERK_Z     20 3 || say "PSC_JERK_Z not accepted: the path will be followed late"
set_retry i ARMING_CHECK   0   || exit 1

say "tuning parameters (defaults = ArduPilot values)"
ITC=${ATC_INPUT_TC:-0.15}
ANG=${ATC_ANG_P:-4.5}
PXY=${PSC_POSXY_P:-1.0}
VXI=${PSC_VELXY_I:-1.0}
PZ=${PSC_POSZ_P:-1.0}
VZ=${PSC_VELZ_P:-5.0}
AZP=${PSC_ACCZ_P:-0.5}
AZI=${PSC_ACCZ_I:-1.0}
set_retry f ATC_INPUT_TC  $ITC || exit 1
set_retry f ATC_ANG_RLL_P $ANG || exit 1
set_retry f ATC_ANG_PIT_P $ANG || exit 1
set_retry f PSC_POSXY_P   $PXY || exit 1
set_retry f PSC_VELXY_I   $VXI || exit 1
set_retry f PSC_POSZ_P    $PZ  || exit 1
set_retry f PSC_VELZ_P    $VZ  || exit 1
set_retry f PSC_ACCZ_P    $AZP || exit 1
set_retry f PSC_ACCZ_I    $AZI || exit 1
say "ATC_INPUT_TC=$ITC ANG_P=$ANG POSXY_P=$PXY VELXY_I=$VXI POSZ_P=$PZ VELZ_P=$VZ ACCZ_P=$AZP ACCZ_I=$AZI"

takeoff_hover "$Z0" || exit 1
if [ "$USE_VIO" = 1 ]; then switch_to_vio || exit 1; fi

say "following the trajectory"
ros2 run vio_launcher traj_follower.py --ros-args -p use_sim_time:=true -p traj:="$TRAJ" \
    -p log:="${FLIGHT_LOGS:-$HOME/ros2_ws/flight_logs}/track_log_$(date +%Y%m%d_%H%M%S)"
rc=$?
[ "$rc" = 0 ] || say "follower exited with code $rc"

if [ "$USE_VIO" = 1 ]; then say "switching back to GPS"; src 1; fi
say "landing"
mode LAND
wait_alt le 0.2 60 || say "did not confirm touchdown, continuing anyway"
sleep 3
stop_recorder
exit "$rc"