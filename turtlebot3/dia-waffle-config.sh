#!/usr/bin/env bash

ROS_VER=jazzy

source /opt/ros/${ROS_VER}/setup.bash
source /home/ros/tb3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-01
export WAFFLE_NO=$(hostname | tr -d -c 0-9)
export ROS_DOMAIN_ID=$WAFFLE_NO

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/${ROS_VER}/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Zenoh related mods:
# export ROS_DISCOVERY_SERVER=127.0.0.1:11811
# export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# read -r -d '' CYCLONEDDS_URI << EOF
# <CycloneDDS>
#   <Domain>
#     <Discovery>
#       <ParticipantIndex>none</ParticipantIndex>
#     </Discovery>
#   </Domain>
# </CycloneDDS>
# EOF
# export CYCLONEDDS_URI

# tb3_bringup() {
#   ZENOH_ID=$(pgrep zenoh)
  
#   if [[ $ZENOH_ID ]]; then
#     pkill zenoh
#   fi

#   echo "Launching a Zenoh Bridge..."
#   sleep 1
#   zenoh-bridge-ros2dds &
  
#   ROS_ARGS=$2

#   echo "Launching ROS [ ros2 launch tuos_tb3_tools ros.launch.py ${ROS_ARGS} ]"
#   sleep 1
#   ros2 launch tuos_tb3_tools ros.launch.py ${ROS_ARGS}
  
# }
