#!/usr/bin/env bash
set -e
. /ws_aic/install/setup.bash
export ROS_HOME=/tmp/phy_portoffset_randomization/triangulation-logging-check-v2-268-204699-w00-204699/trial_0268/ros_home
mkdir -p "$ROS_HOME"
unset LIBGL_ALWAYS_SOFTWARE __EGL_VENDOR_LIBRARY_FILENAMES
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=40
export GZ_PARTITION=phy_triangulation-logging-check-v2-268-204699_w00
export IGN_PARTITION=phy_triangulation-logging-check-v2-268-204699_w00
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export AIC_RESULTS_DIR=/tmp/phy_portoffset_randomization/triangulation-logging-check-v2-268-204699-w00-204699/trial_0268/aic_results
export AIC_PORTOFFSET_RUN_ID=triangulation-logging-check-v2-268-204699-w00-204699
export ZENOH_ROUTER_CONFIG_URI=/aic_zenoh_config.json5
export ZENOH_CONFIG_OVERRIDE='mode="router";listen/endpoints=["tcp/[::]:7600"];connect/endpoints=[];routing/router/peers_failover_brokering=true;transport/shared_memory/enabled=false'
ros2 run rmw_zenoh_cpp rmw_zenohd &
router_pid=$!
cleanup_router() { kill -SIGINT "$router_pid" 2>/dev/null || true; wait "$router_pid" 2>/dev/null || true; }
trap cleanup_router EXIT
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:7600"];transport/shared_memory/enabled=false'
ros2 launch aic_bringup aic_gz_bringup.launch.py spawn_task_board:=false spawn_cable:=false ground_truth:=true start_aic_engine:=true aic_engine_config_file:=/tmp/phy_portoffset_randomization/triangulation-logging-check-v2-268-204699-w00-204699/trial_0268/engine_config.yaml world_file:=/tmp/phy_portoffset_randomization/triangulation-logging-check-v2-268-204699-w00-204699/trial_0268/randomized_world.sdf gazebo_gui:=false launch_rviz:=false description_file:=/home/swlinux/Desktop/workspace/aic-physic/evidence/triangulation/logging-check-v2/job_268/attempt_000/annotation.urdf.xacro ros_gz_bridge_config_file:=/tmp/phy_portoffset_randomization/triangulation-logging-check-v2-268-204699-w00-204699/trial_0268/engine_config_bridge.yaml
