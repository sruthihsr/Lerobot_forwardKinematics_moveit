#!/usr/bin/env bash
# Single entry point: builds, runs pre-flight checks, then launches MoveIt + the IK loop around the cylinder.
#
#   ./run.sh                              mock arm, plan-only (safe anywhere, no hardware)
#   ./run.sh --execute                    mock arm, actually loops
#   ./run.sh real                         real arm connected READ-ONLY (dry run: nothing moves)
#   ./run.sh real --live --execute        real arm, MOVES (asks you to confirm)
#
# Run ./run.sh --help for all options. Details: README.md, docs/HARDWARE_BRINGUP.md
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./run.sh [mock|real] [options]

  mock (default)   ros2_control mock hardware -- no arm needed
  real             Feetech bridge on the serial port. Read-only unless --live

Options:
  --execute        run the loop (default: plan-only pre-flight, nothing moves)
  --live           real only: dry_run:=False -- the bridge WILL MOVE THE ARM
  --x X --y Y      cylinder centre in the base frame, metres (+x = arm's left, -y = forward)
                   default 0.0708 -0.2731 -- MEASURE YOURS (docs/HARDWARE_BRINGUP.md)
  --clearance M    gripper height above the padded cylinder top when crossing   (default 0.07)
  --speed R        peak joint speed, rad/s                                       (default 0.2)
  --cycles N       loops to run, 0 = until Ctrl+C                                (default 0)
  --pause S        seconds held at each point                                    (default 2.0)
  --points FILE    YAML loop points (default config/loop_points.yaml)
  --port DEV       serial port                                                   (default /dev/ttyACM0)
  --yes            skip the live-hardware confirmation prompt
  --no-build       skip colcon build
  --force          skip the 'another stack is running' check (only sensible with a different ROS_DOMAIN_ID)
  -h, --help       this text
EOF
}

MODE=mock; EXECUTE=False; LIVE=False; YES=0; BUILD=1; FORCE=0
X=0.0708; Y=-0.2731; CLEARANCE=0.07; SPEED=0.2; CYCLES=0; PAUSE=2.0; POINTS=""; PORT=/dev/ttyACM0

while [[ $# -gt 0 ]]; do
  case "$1" in
    mock|real) MODE=$1 ;;
    --execute) EXECUTE=True ;;
    --live) LIVE=True ;;
    --x) X=$2; shift ;;
    --y) Y=$2; shift ;;
    --clearance) CLEARANCE=$2; shift ;;
    --speed) SPEED=$2; shift ;;
    --cycles) CYCLES=$2; shift ;;
    --pause) PAUSE=$2; shift ;;
    --points) POINTS=$2; shift ;;
    --port) PORT=$2; shift ;;
    --yes) YES=1 ;;
    --no-build) BUILD=0 ;;
    --force) FORCE=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ "$MODE" == mock && "$LIVE" == True ]] && { echo "--live only applies to 'real'" >&2; exit 2; }

# rclpy is built for the system Python 3.12; a conda 'base' python first on PATH breaks it.
export PATH=/usr/bin:$PATH

WS=${ROS2_WS:-$HOME/ros2_ws}
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS 2 Jazzy not found at /opt/ros/jazzy" >&2; exit 1; }
[[ -d "$WS/src" ]] || { echo "workspace not found at $WS (set ROS2_WS)" >&2; exit 1; }
set +u   # ROS setup scripts reference unset variables
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ $BUILD -eq 1 ]]; then
  (cd "$WS" && colcon build --packages-select lerobot_description lerobot_controller lerobot_hardware lerobot_moveit lerobot_ik_demo 2>&1 | tail -3)
fi
[[ -f "$WS/install/setup.bash" ]] || { echo "$WS/install/setup.bash missing -- build failed?" >&2; exit 1; }
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# One stack at a time: two launches = two bridges fighting over the serial port (crashes both) or a mixed ROS graph.
if [[ $FORCE -eq 0 ]] && { pgrep -f "ros2 launch (lerobot_moveit|lerobot_ik_demo)" >/dev/null || pgrep -f "so101_hardware_bridge" >/dev/null; }; then
  echo "ERROR: another lerobot stack (move_group / hardware bridge) is already running. Stop it first (Ctrl+C in its terminal)." >&2
  exit 1
fi

DRY_RUN=True
if [[ "$MODE" == real ]]; then
  [[ -e "$PORT" ]] || { echo "ERROR: $PORT not found -- is the arm plugged in and powered?" >&2; exit 1; }
  if command -v lsof >/dev/null && lsof "$PORT" >/dev/null 2>&1; then
    echo "ERROR: $PORT is in use by another process." >&2; exit 1
  fi
  [[ "$LIVE" == True ]] && DRY_RUN=False
fi

echo "mode=$MODE dry_run=$DRY_RUN execute=$EXECUTE cylinder=($X, $Y) clearance=$CLEARANCE speed=$SPEED cycles=$CYCLES"

if [[ "$MODE" == real && "$DRY_RUN" == False && "$EXECUTE" == True && $YES -eq 0 ]]; then
  cat <<EOF

  ================= LIVE HARDWARE: THE ARM WILL MOVE =================
   * cylinder must stand at x=$X, y=$Y (base frame) -- the plan is made for that spot
   * nothing else in the arm's swing; the base sweeps up to ~100 degrees
   * start pose must be clear of the cylinder, the arm holds position with torque on
   * a hand on the power switch. Ctrl+C stops the loop; the arm then holds its last pose
   * the tip lands 1-5 cm off target (gravity sag) -- see docs/ARCHITECTURE.md
  ====================================================================
EOF
  read -r -p "Type 'yes' to continue: " ans
  [[ "$ans" == yes ]] || { echo "aborted"; exit 1; }
fi

ARGS=(hardware:="$MODE" dry_run:="$DRY_RUN" port:="$PORT" execute:="$EXECUTE"
      object_x:="$X" object_y:="$Y" clearance:="$CLEARANCE" speed:="$SPEED" cycles:="$CYCLES" pause:="$PAUSE")
[[ -n "$POINTS" ]] && ARGS+=(points:="$POINTS")

exec ros2 launch lerobot_ik_demo cylinder_loop.launch.py "${ARGS[@]}"
