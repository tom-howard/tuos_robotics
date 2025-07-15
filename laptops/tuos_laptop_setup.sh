# Bash profile with ROS settings for the Turtlebot3 robot

# ROS settings for TurtleBot3 (localhost)
source /opt/ros/jazzy/setup.bash
source $HOME/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle

# Check if waffle_number file exists, if not copy laptop_number to waffle_number
if [ ! -f $HOME/.diamond/waffle_number ]; then
    cp /home/ros/laptop_number $HOME/.diamond/waffle_number
fi

export WAFFLE_NO=$(cat $HOME/.diamond/waffle_number 2>/dev/null)

# Check the content of robot_mode file and set RDS and LHOST_ONLY variables accordingly
if [ ! -f $HOME/.diamond/robot_mode ]; then
    echo "robot" > $HOME/.diamond/robot_mode
elif grep -qi "robot" $HOME/.diamond/robot_mode; then
    RDS="dia-waffle$WAFFLE_NO:11811;dia-waffle$WAFFLE_NO:11888"
    LHOST_ONLY=0
    SUPER_CLIENT=TRUE
elif grep -qi "sim" $HOME/.diamond/robot_mode; then
    RDS=""
    LHOST_ONLY=1
    SUPER_CLIENT=FALSE
    export ROS_DOMAIN_ID=$(cat /home/ros/laptop_number)
else
    RDS="dia-waffle$WAFFLE_NO:11811;dia-waffle$WAFFLE_NO:11888"
    LHOST_ONLY=0
    SUPER_CLIENT=TRUE
fi

# export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=$WAFFLE_NO

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

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

colcon() {
    # If the first argument is "build", check the current directory
    if [[ "$1" == "build" ]]; then
        if [ "$PWD" != "$HOME/ros2_ws" ]; then
            echo "Error: 'colcon build' must be run from $HOME/ros2_ws."
            return 1
        fi
    fi

    # Execute the actual colcon command with all provided arguments
    command colcon "$@"
}
