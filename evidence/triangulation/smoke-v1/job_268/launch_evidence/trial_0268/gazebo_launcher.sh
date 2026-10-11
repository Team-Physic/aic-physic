#!/usr/bin/env bash
set -e
. /ws_aic/install/setup.bash
export ROS_HOME=/tmp/phy_portoffset_randomization/triangulation-smoke-v1-268-27737-w00-27737/trial_0268/ros_home
mkdir -p "$ROS_HOME"
export LIBGL_ALWAYS_SOFTWARE=1
export __EGL_VENDOR_LIBRARY_FILENAMES="$ROS_HOME/no-egl.json"
export LP_NUM_THREADS=4
Xvfb -displayfd 3 -screen 0 1280x1024x24 -nolisten tcp 3>"$ROS_HOME/display" >"$ROS_HOME/xvfb.log" 2>&1 &
xvfb_pid=$!
for attempt in {1..50}; do [ -s "$ROS_HOME/display" ] && break; sleep 0.1; done
[ -s "$ROS_HOME/display" ] || { kill "$xvfb_pid"; exit 1; }
export DISPLAY=$(cat "$ROS_HOME/display" | sed "s/^/:/")
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=40
export GZ_PARTITION=phy_triangulation-smoke-v1-268-27737_w00
export IGN_PARTITION=phy_triangulation-smoke-v1-268-27737_w00
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export AIC_RESULTS_DIR=/tmp/phy_portoffset_randomization/triangulation-smoke-v1-268-27737-w00-27737/trial_0268/aic_results
export AIC_PORTOFFSET_RUN_ID=triangulation-smoke-v1-268-27737-w00-27737
export ZENOH_ROUTER_CONFIG_URI=/aic_zenoh_config.json5
export ZENOH_CONFIG_OVERRIDE='mode="router";listen/endpoints=["tcp/[::]:7600"];connect/endpoints=[];routing/router/peers_failover_brokering=true;transport/shared_memory/enabled=false'
ros2 run rmw_zenoh_cpp rmw_zenohd &
router_pid=$!
cleanup_router() { kill -SIGINT "$router_pid" 2>/dev/null || true; wait "$router_pid" 2>/dev/null || true; }
trap 'cleanup_router; kill "$xvfb_pid" 2>/dev/null || true; wait "$xvfb_pid" 2>/dev/null || true' EXIT
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:7600"];transport/shared_memory/enabled=false'
ros2 launch aic_bringup aic_gz_bringup.launch.py spawn_task_board:=false spawn_cable:=false ground_truth:=true start_aic_engine:=true aic_engine_config_file:=/tmp/phy_portoffset_randomization/triangulation-smoke-v1-268-27737-w00-27737/trial_0268/engine_config.yaml world_file:=/tmp/phy_portoffset_randomization/triangulation-smoke-v1-268-27737-w00-27737/trial_0268/randomized_world.sdf launch_rviz:=false description_file:=/home/swlinux/Desktop/workspace/aic-physic/ws_aic/src/phy/phy_data_collection/config/ur_gz_depth.urdf.xacro ros_gz_bridge_config_file:=/tmp/phy_portoffset_randomization/triangulation-smoke-v1-268-27737-w00-27737/trial_0268/engine_config_bridge.yaml
