# cart_env.sh -- source this (do NOT run it) to get a shell set up for driving
# the cart:  source ~/1Mapping/cart_env.sh
#
# 1Mapping and 2Localize_get_path both ship golfcart_low_level, ouster_ros and
# ublox_gps, and in ROS 2 the workspace sourced LAST wins. For driving,
# 2Localize has to win: it holds lidar_localization_ros2 and it is the only
# ublox_gps carrying ublox_gps_node_zed_f9r-launch.py. Sourcing 1Mapping last
# is what produces "file 'ublox_gps_node_zed_f9r-launch.py' was not found".
#
# For MAPPING (LIO-SAM) source 1Mapping last instead -- the opposite order.

source /opt/ros/humble/setup.bash
[ -f ~/1Mapping/install/setup.bash ] && source ~/1Mapping/install/setup.bash
source ~/2Localize_get_path/install/setup.bash

for _pkg in lidar_localization_ros2 ublox_gps ouster_ros golfcart_low_level; do
    _prefix=$(ros2 pkg prefix "$_pkg" 2>/dev/null)
    case "$_prefix" in
        "$HOME"/2Localize_get_path/*) printf '  ok  %-24s <- 2Localize_get_path\n' "$_pkg" ;;
        "$HOME"/1Mapping/*)           printf '  !!  %-24s <- 1Mapping (shadowing)\n' "$_pkg" ;;
        '')                           printf '  !!  %-24s not found\n' "$_pkg" ;;
        *)                            printf '  ??  %-24s <- %s\n' "$_pkg" "$_prefix" ;;
    esac
done
unset _pkg _prefix
