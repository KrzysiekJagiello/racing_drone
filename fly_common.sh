#!/usr/bin/env bash
# Helpers for the flight scripts. Source it, do not run it.

set +u      # ROS's setup.bash reads unset variables
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

say()   { echo "[fly] $*"; }
call()  { timeout 10 ros2 service call "$@" 2>&1 | grep -o 'success=[A-Za-z]*\|mode_sent=[A-Za-z]*\|result=[0-9]*' | tr '\n' ' '; echo; }
seti()  { call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{force_set: false, param_id: '$1', value: {type: 2, integer_value: $2}}"; }
setf()  { call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{force_set: false, param_id: '$1', value: {type: 3, double_value: $2}}"; }
mode()  { call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: '$1'}"; }
src()   { call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 42007, param1: $1}"; }
pos()   { timeout 3 ros2 topic echo /mavros/local_position/pose --once 2>/dev/null \
            | awk '/position:/{f=1} f&&/x:/{x=$2} f&&/y:/{y=$2} f&&/z:/{print x, y, $2; exit}'; }
alt()   { pos | awk '{print $3}'; }

wait_alt() {   # wait_alt <ge|le> <metres> <timeout s>
  local t=0
  while [ "$t" -lt "$3" ]; do
    local z; z=$(alt)
    if [ -n "$z" ] && awk -v z="$z" -v lim="$2" -v op="$1" 'BEGIN{exit !((op=="ge" && z>=lim) || (op=="le" && z<=lim))}'; then return 0; fi
    sleep 1; t=$((t+1))
  done
  return 1
}

wait_fcu() { until timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; do sleep 1; done; }

set_retry() {   # set_retry <i|f> <name> <value> [tries=30]: retries until MAVROS has the parameter list
  local n=0 out tries="${4:-30}"
  while :; do
    if [ "$1" = i ]; then out=$(seti "$2" "$3"); else out=$(setf "$2" "$3"); fi
    if echo "$out" | grep -q "success=True"; then return 0; fi
    n=$((n+1))
    if [ "$n" -ge "$tries" ]; then say "could not set $2 after $tries tries"; return 1; fi
    [ "$n" = 1 ] && say "parameter $2 not accepted yet (MAVROS still loading the parameter list?), retrying..."
    sleep 2
  done
}

wait_gps() {   # wait for a GPS fix (the EKF needs one on GPS)
  local t=0 s
  say "waiting for a GPS fix"
  while [ "$t" -lt 120 ]; do
    s=$(timeout 3 ros2 topic echo /mavros/global_position/global --once 2>/dev/null | awk '/^status:/{f=1;next} f&&/status:/{print $2; exit}')
    if [ -n "$s" ] && [ "$s" -ge 0 ] 2>/dev/null; then return 0; fi
    sleep 1; t=$((t+1))
  done
  say "no GPS fix after 120 s"
  return 1
}

is_armed() { timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "armed: true"; }
climbing() {   # above 0.5 m: the takeoff worked even if its reply was lost
  local z; z=$(alt); [ -n "$z" ] && awk -v z="$z" 'BEGIN{exit !(z>0.5)}'
}

takeoff_hover() {   # takeoff_hover <height m>: GUIDED, arm, climb, hover 5 s
  say "GUIDED, arm, takeoff to $1 m"
  mode GUIDED
  local n=0
  until is_armed || call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" | grep -q "success=True"; do
    n=$((n+1))
    if [ "$n" -ge 30 ]; then say "could not arm after 30 tries: read the AP: lines in the sim terminal for the reason"; return 1; fi
    say "arm refused (EKF still starting?), retrying ($n/30)..."
    sleep 2
    mode GUIDED
  done
  n=0
  until climbing || call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: $1}" | grep -q "success=True" || { sleep 1; climbing; }; do
    n=$((n+1))
    if [ "$n" -ge 8 ]; then say "takeoff refused 8 times, landing"; mode LAND; return 1; fi
    say "takeoff refused, retrying ($n/8)..."
    sleep 1
  done
  local lim; lim=$(awk -v z="$1" 'BEGIN{print z-0.3}')
  if ! wait_alt ge "$lim" 40; then say "never reached $lim m, landing"; mode LAND; return 1; fi
  say "at height, hovering 5 s"
  sleep 5
}

switch_to_vio() {   # checks the vision data, switches the EKF to source set 2, lands on GPS if the position jumps
  local max_jump=0.5 ok=0 before after jump
  say "checking that vision data is flowing"
  for _ in $(seq 1 20); do
    if timeout 3 ros2 topic echo /mavros/vision_pose/pose --once >/dev/null 2>&1; then ok=1; break; fi
  done
  if [ "$ok" = 0 ]; then say "no /mavros/vision_pose/pose: OpenVINS not initialised or the relay is not running. Staying on GPS, landing."; mode LAND; return 1; fi
  before=$(pos); say "position before switch: $before"
  src 2; sleep 4
  after=""
  for _ in $(seq 1 6); do after=$(pos); [ -n "$after" ] && break; done   # up to ~18 s
  if [ -z "$after" ]; then
    say "EKF has NO position on VIO (local_position stopped). Switching back to GPS and landing."
    say "check in window vio: is OpenVINS still tracking? and: ros2 topic hz /mavros/vision_pose/pose"
    src 1; sleep 2; mode LAND; return 1
  fi
  say "position after switch:  $after"
  jump=$(awk -v a="$before" -v b="$after" 'BEGIN{split(a,p," ");split(b,q," ");print sqrt((p[1]-q[1])^2+(p[2]-q[2])^2+(p[3]-q[3])^2)}')
  say "jump at switch: $jump m"
  if awk -v j="$jump" -v m="$max_jump" 'BEGIN{exit !(j>m)}'; then
    say "jump larger than $max_jump m: switching back to GPS and landing"
    src 1; mode LAND; return 1
  fi
}

stop_recorder() {
  if tmux has-session -t vio 2>/dev/null; then
    say "stopping the recorder, results appear in the 'rec' window"
    tmux send-keys -t vio:rec C-c
  else
    say "done. Press Ctrl-C in the recorder terminal to get the metrics."
  fi
}