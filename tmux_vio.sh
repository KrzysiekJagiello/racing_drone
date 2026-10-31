#!/usr/bin/env bash
# Start the VIO sim stack in one tmux session.
#   ./tmux_vio.sh          start, or re-attach if it is running
#   ./tmux_vio.sh stop     Ctrl-C every pane, then kill the session
# Windows: sim | vio (OpenVINS | relay) | rec (recorder) | fly (free shell)
# Keys: Ctrl-b n / p = next / previous window, Ctrl-b d = detach. Mouse scrolling is on.
# Recorder reference: Gazebo truth /ground_truth/odom. GT=/mavros/local_position/odom uses the EKF instead.

set -u
WS="${WS:-$HOME/ros2_ws}"
S=vio
GT="${GT:-/ground_truth/odom}"
SRC="source /opt/ros/humble/setup.bash; source $WS/install/setup.bash"

if [ "${1:-}" = "stop" ]; then
  tmux has-session -t "$S" 2>/dev/null || { echo "no '$S' session"; exit 0; }
  for p in $(tmux list-panes -s -t "$S" -F '#{pane_id}'); do tmux send-keys -t "$p" C-c; done
  echo "sent Ctrl-C to all panes, waiting for processes to exit..."
  sleep 6
  tmux kill-session -t "$S"
  exit 0
fi

command -v tmux >/dev/null || { echo "tmux is not installed: sudo apt-get install -y tmux"; exit 1; }

if tmux has-session -t "$S" 2>/dev/null; then
  echo "session '$S' already running, attaching (stop it with: $0 stop)"
  exec tmux attach -t "$S"
fi

# commands run inside the panes: wait for the sim before starting
W_IMU='until ros2 topic list 2>/dev/null | grep -qx /iris_rgb/imu; do sleep 1; done'
W_FCU='until ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; do sleep 1; done'

LOGDIR="${FLIGHT_LOGS:-$WS/flight_logs}/logs_open_vins"; mkdir -p "$LOGDIR"     # OpenVINS console output per session
tmux new-session  -d -s "$S" -n sim -c "$WS"
tmux set-option   -t "$S" mouse on
tmux set-option   -t "$S" history-limit 50000

# sim: Gazebo, SITL, MAVROS, bridge
tmux send-keys -t "$S:sim" "$SRC; ./start.sh" C-m

# vio: OpenVINS (left), MAVROS relay (right)
tmux new-window -t "$S:" -n vio -c "$WS"
tmux send-keys  -t "$S:vio.0" "export LOGDIR=$LOGDIR; $SRC; echo 'waiting for sim...'; $W_IMU; $W_FCU; sleep 3; \
ros2 launch ov_msckf subscribe.launch.py \
config_path:=\$(ros2 pkg prefix vio_launcher)/share/vio_launcher/config/openvins/iris_rgb/estimator_config.yaml \
max_cameras:=1 use_stereo:=false rviz_enable:=false 2>&1 | tee \"\$LOGDIR/openvins_\$(date +%Y%m%d_%H%M%S).log\"" C-m
tmux split-window -h -t "$S:vio" -c "$WS"
tmux send-keys  -t "$S:vio.1" "$SRC; echo 'waiting for sim...'; $W_IMU; $W_FCU; \
ros2 run vio_launcher vio_to_mavros.py --ros-args -p use_sim_time:=true" C-m

# rec: recorder (Ctrl-C to stop; the flight scripts stop it after landing)
tmux new-window -t "$S:" -n rec -c "$WS"
tmux send-keys  -t "$S:rec" "$SRC; echo 'waiting for sim...'; $W_IMU; python3 vio_vs_gt.py --gt $GT" C-m

# fly: free shell for the flight scripts
tmux new-window -t "$S:" -n fly -c "$WS"
tmux send-keys  -t "$S:fly" "$SRC; clear; echo 'when OpenVINS has printed its init in window vio:'; echo '  ./fly_square.sh 1          one lap, GPS'; echo '  ./fly_square.sh 1 --vio    one lap, EKF switched to VIO'" C-m

tmux select-window -t "$S:fly"
exec tmux attach -t "$S"