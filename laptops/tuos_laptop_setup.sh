# Bash profile with ROS settings for the Turtlebot3 robot

# ROS settings for TurtleBot3 (localhost)
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle

# Check if waffle_number file exists, if not copy laptop_number to waffle_number
if [ ! -f $HOME/.tuos/waffle_number ]; then
    cp /home/laptop/laptop_number $HOME/.tuos/waffle_number
fi

export WAFFLE_NO=$(cat $HOME/.tuos/waffle_number 2>/dev/null)

# Check the content of robot_mode file and set RDS and LHOST_ONLY variables accordingly
if [ ! -f $HOME/.tuos/robot_mode ]; then
    echo "robot" > $HOME/.tuos/robot_mode
elif grep -qi "robot" $HOME/.tuos/robot_mode; then
    RDS="dia-waffle$WAFFLE_NO:11811;dia-waffle$WAFFLE_NO:11888"
    LHOST_ONLY=0
    SUPER_CLIENT=TRUE
elif grep -qi "sim" $HOME/.tuos/robot_mode; then
    RDS=""
    LHOST_ONLY=1
    SUPER_CLIENT=FALSE
    export ROS_DOMAIN_ID=$(cat /home/laptop/laptop_number)
    source /usr/share/gazebo/setup.bash
else
    RDS="dia-waffle$WAFFLE_NO:11811;dia-waffle$WAFFLE_NO:11888"
    LHOST_ONLY=0
    SUPER_CLIENT=TRUE
fi

# Export the updated variables for ROS settings
# export ROS_DISCOVERY_SERVER=$RDS
export ROS_HOSTNAME=$(hostname)
# export ROS_SUPER_CLIENT=$SUPER_CLIENT

# Mods for Zenoh
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=$WAFFLE_NO

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

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
