#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /home/ros/tb3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-01
export WAFFLE_NO=$(hostname | tr -d -c 0-9)
export ROS_DOMAIN_ID=$WAFFLE_NO

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Zenoh related mods:
# export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

read -r -d '' CYCLONEDDS_URI << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="0">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
      <FragmentSize>4000B</FragmentSize>
      <Transport>udp</Transport>
    </General>
    <Discovery>
      <Peers>
        <Peer address="localhost"/>
        <Peer address="`hostname`"/>
      </Peers>
      <MaxAutoParticipantIndex>1000</MaxAutoParticipantIndex>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
    <Internal>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>info</Verbosity>
      <OutputFile>stdout</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
export CYCLONEDDS_URI

tb3_bringup() {
  ZENOH_ID=$(pgrep zenoh)
  if [[ $ZENOH_ID ]]; then
    if [[ "$1" == "restart" || "$1" == "r" ]]; then
      echo "Killing an existing Zenoh bridge (PID: $ZENOH_ID)."
      pkill zenoh
      echo "Launching a Zenoh Bridge..."
      sleep 3
      zenoh-bridge-ros2dds &
    else
      echo "A Zenoh bridge is already running (PID: $ZENOH_ID)."
    fi
  else
    echo "Launching a Zenoh Bridge..."
    sleep 3
    zenoh-bridge-ros2dds &
  fi

  ROS_ARGS=$2

  echo "Launching ROS [ ros2 launch tuos_tb3_tools ros.launch.py ${ROS_ARGS} ]"
  sleep 3
  ros2 launch tuos_tb3_tools ros.launch.py ${ROS_ARGS}
  
}
