#!/usr/bin/env bash
#
# start_cart.sh -- brings the golf cart stack up in dependency order, in one
# terminal, and takes it all back down on Ctrl+C.
#
# Stages, in order (each waits for the previous one to actually be alive, not
# just launched -- localization matching against a sensor that has not started
# streaming is how the pose ends up frozen from the first scan):
#
#   1. can0          vehicle CAN bus                     (needs sudo)
#   2. low_level     ros_can_bridge, steering_node
#   3. ouster        OS lidar driver  -> /ouster/points
#   4. gnss          ZED-F9R + ntrip_client -> /fix, /rtcm
#   5. localization  NDT against GlobalMap.pcd -> /pcl_pose
#   6. drive         waypoint follower -> /cmd_vel        (ONLY with --drive)
#
# Stage 6 is the one that makes the cart move, so it is opt-in. Everything
# else only reads sensors and publishes a pose.
#
# Usage:
#   bash ~/1Mapping/start_cart.sh                 sensors + localization, cart does not move
#   bash ~/1Mapping/start_cart.sh --drive         also start the waypoint follower
#   bash ~/1Mapping/start_cart.sh --dry-run       start the follower with dry_run:=true
#   bash ~/1Mapping/start_cart.sh --rviz          also open rviz2 (do this on a desk machine)
#   bash ~/1Mapping/start_cart.sh --no-can        skip CAN + low_level (bench testing)
#   bash ~/1Mapping/start_cart.sh --no-gnss       skip F9R + ntrip
#
set -uo pipefail

WS=/home/myuser/2Localize_get_path
MAP_WS=/home/myuser/1Mapping
SIM=$WS/src/sim
LOG_DIR=/tmp/start_cart/$(date +%Y%m%d_%H%M%S)
LOW_LEVEL_PIDFILE=/tmp/golfcart_low_level_launch.pid

WANT_DRIVE=0
WANT_DRY_RUN=0
WANT_RVIZ=false
WANT_CAN=1
WANT_GNSS=1

for arg in "$@"; do
    case "$arg" in
        --drive)    WANT_DRIVE=1 ;;
        --dry-run)  WANT_DRIVE=1; WANT_DRY_RUN=1 ;;
        --rviz)     WANT_RVIZ=true ;;
        --no-can)   WANT_CAN=0 ;;
        --no-gnss)  WANT_GNSS=0 ;;
        -h|--help)  awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' "$0"; exit 0 ;;
        *) echo "unknown option: $arg (try --help)"; exit 2 ;;
    esac
done

mkdir -p "$LOG_DIR"
PIDS=()
NAMES=()

say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '    \033[32mok\033[0m  %s\n' "$*"; }
warn() { printf '    \033[33m!!\033[0m  %s\n' "$*"; }
die()  { printf '    \033[31mxx\033[0m  %s\n' "$*"; shutdown_all; exit 1; }

shutdown_all() {
    trap '' INT TERM
    say "shutting down"
    # reverse order, so the controller stops commanding before its inputs vanish
    for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
        local pid=${PIDS[$i]}
        kill -0 "$pid" 2>/dev/null || continue
        printf '    stopping %s (pid %s)\n' "${NAMES[$i]}" "$pid"
        kill -INT "$pid" 2>/dev/null
    done
    for _ in $(seq 1 20); do
        local alive=0
        for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
        [ "$alive" -eq 0 ] && break
        sleep 0.5
    done
    for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null; done
    rm -f "$LOW_LEVEL_PIDFILE"
    echo "    logs kept in $LOG_DIR"
}
trap 'echo; shutdown_all; exit 0' INT TERM

# start <name> <logfile-stem> -- <command...>
start() {
    local name=$1 stem=$2; shift 3
    "$@" >"$LOG_DIR/$stem.log" 2>&1 &
    local pid=$!
    PIDS+=("$pid"); NAMES+=("$name")
    sleep 0.5
    if ! kill -0 "$pid" 2>/dev/null; then
        warn "$name died immediately -- tail of $LOG_DIR/$stem.log:"
        tail -n 15 "$LOG_DIR/$stem.log" | sed 's/^/        /'
        return 1
    fi
    ok "$name started (pid $pid, log $stem.log)"
}

# wait_topic <topic> <timeout-s> -- returns 1 if nothing is published in time
wait_topic() {
    local topic=$1 timeout=$2 waited=0
    while [ "$waited" -lt "$timeout" ]; do
        if timeout 2 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
            ok "$topic is publishing"
            return 0
        fi
        sleep 2
        waited=$(( waited + 2 ))
        printf '    waiting for %s (%ss/%ss)\r' "$topic" "$waited" "$timeout"
    done
    printf '\n'
    return 1
}

# ROS setup scripts read unset variables (AMENT_TRACE_SETUP_FILES and friends),
# so -u has to come off around them or sourcing aborts the script.
#
# ORDER MATTERS. 1Mapping and 2Localize_get_path both ship golfcart_low_level,
# ouster_ros and ublox_gps, and the workspace sourced LAST wins. 2Localize has
# to go last: it is the one holding lidar_localization_ros2 and the only
# ublox_gps that has ublox_gps_node_zed_f9r-launch.py at all. Sourcing
# 1Mapping last (as the old by-hand command did) makes `ros2 launch ublox_gps
# ublox_gps_node_zed_f9r-launch.py` fail with "file not found", because it
# resolves ublox_gps to the 1Mapping copy that has no such launch file.
set +u
source /opt/ros/humble/setup.bash
[ -f "$MAP_WS/install/setup.bash" ] && source "$MAP_WS/install/setup.bash"
[ -f "$WS/install/setup.bash" ] || { set -u; die "$WS/install is missing -- colcon build first"; }
source "$WS/install/setup.bash"
set -u
say "workspaces sourced, logs -> $LOG_DIR"

# Show where each package actually resolves. Silent shadowing between these two
# overlay workspaces is the kind of thing that costs an afternoon on the cart.
for pkg in lidar_localization_ros2 ublox_gps ouster_ros golfcart_low_level; do
    prefix=$(ros2 pkg prefix "$pkg" 2>/dev/null) || { warn "$pkg not found in either workspace"; continue; }
    case "$prefix" in
        "$WS"/*)     ok   "$pkg <- 2Localize_get_path" ;;
        "$MAP_WS"/*) warn "$pkg <- 1Mapping (shadowing 2Localize_get_path)" ;;
        *)           warn "$pkg <- $prefix" ;;
    esac
done

# ---------------------------------------------------------------- 1. CAN bus
if [ "$WANT_CAN" -eq 1 ]; then
    say "1/6  CAN bus"
    if ip link show can0 2>/dev/null | grep -q "state UP"; then
        ok "can0 already up"
    elif bash "$WS/src/golfcart_low_level/scripts/setup_can0.sh" >"$LOG_DIR/can0.log" 2>&1; then
        ok "can0 configured"
    else
        warn "setup_can0.sh failed -- see $LOG_DIR/can0.log"
        die "no CAN bus; re-run with --no-can to bring the rest up anyway"
    fi

    # -------------------------------------------------------- 2. low level
    say "2/6  low level (CAN bridge, steering, odom_bridge)"
    if [ -f "$LOW_LEVEL_PIDFILE" ] && kill -0 "$(cat "$LOW_LEVEL_PIDFILE")" 2>/dev/null; then
        die "low_level already running (pid $(cat "$LOW_LEVEL_PIDFILE")); stop_low_level.sh first"
    fi
    start "low_level" low_level -- ros2 launch golfcart_low_level low_level.launch.py \
        || die "low_level would not start"
    echo "${PIDS[-1]}" > "$LOW_LEVEL_PIDFILE"
    # low_level.launch.py currently starts ros_can_bridge and steering_node
    # ONLY -- there is no odom_bridge in the tree and nothing publishes
    # /odometry, so localization.yaml's use_odom:true has no data behind it and
    # NDT gets no motion prior between scans. Checked, not assumed, because a
    # missing prior is what turns one dropped scan into a multi-second freeze.
    wait_topic /odometry 10 \
        || warn "/odometry silent -- NDT will run with NO motion prior (no odom_bridge in this build)"
else
    say "1-2/6  CAN + low level SKIPPED (--no-can); /odometry will not be published"
fi

# ------------------------------------------------------------- 3. ouster
say "3/6  ouster lidar"
start "ouster" ouster -- ros2 launch ouster_ros driver.launch.py \
    || die "ouster driver would not start"
wait_topic /ouster/points 45 \
    || die "no /ouster/points after 45s -- check the sensor at 192.168.1.100 (ping it)"

# --------------------------------------------------------------- 4. GNSS
if [ "$WANT_GNSS" -eq 1 ]; then
    say "4/6  GNSS (ZED-F9R + NTRIP)"
    if ls /dev/ttyACM* >/dev/null 2>&1; then
        start "ublox_f9r" ublox -- ros2 launch ublox_gps ublox_gps_node_zed_f9r-launch.py \
            && start "ntrip" ntrip -- ros2 launch ntrip_client ntrip_client-launch.py
        wait_topic /ublox_gps_node/fix 25 || warn "no GNSS fix yet (cold start can take a while)"
    else
        warn "no /dev/ttyACM* -- C102-F9R not enumerated, skipping GNSS"
        warn "check: dmesg | tail -20   and whether it came up as /dev/ttyUSB*"
    fi
else
    say "4/6  GNSS SKIPPED (--no-gnss)"
fi

# ------------------------------------------------------- 5. localization
say "5/6  localization"
start "localization" localization -- ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
    use_rviz:="$WANT_RVIZ" \
    || die "localization would not start"
# the node loads a 5.8M-point map and builds its kd-tree on the first align,
# which took ~1.5 s on a dev box -- give the first pose room to appear
wait_topic /pcl_pose 90 || die "no /pcl_pose after 90s -- see $LOG_DIR/localization.log"

# ------------------------------------------------------------- 6. driving
if [ "$WANT_DRIVE" -eq 1 ]; then
    if [ "$WANT_DRY_RUN" -eq 1 ]; then
        say "6/6  waypoint follower (DRY RUN -- nothing reaches /cmd_vel)"
        DRY=true
    else
        say "6/6  waypoint follower -- THE CART WILL MOVE"
        DRY=false
        printf '    \033[1;31mhand on the e-stop. Enter to go, Ctrl+C to abort: \033[0m'
        read -r
    fi
    start "adaptive_ld" adaptive_ld -- python3 "$SIM/adaptive_ld_node_fixed.py"
    start "follower" follower -- python3 "$SIM/pagard_adaptive_fixed.py" \
        --ros-args -p dry_run:="$DRY" \
        || die "waypoint follower would not start"
else
    say "6/6  waypoint follower NOT started (pass --drive to enable)"
fi

say "up -- Ctrl+C to stop everything"
printf '    pause driving:  bash %s/scripts/pause_driving.sh\n' "$WS"
printf '    watch the pose: ros2 topic hz /pcl_pose\n'
printf '    logs:           %s\n\n' "$LOG_DIR"

# exit as soon as any stage dies, so a crashed node does not leave a half-stack up
while true; do
    for i in "${!PIDS[@]}"; do
        if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
            warn "${NAMES[$i]} exited -- bringing the rest down"
            shutdown_all
            exit 1
        fi
    done
    sleep 2
done
