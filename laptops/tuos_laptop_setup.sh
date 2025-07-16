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
export WAFFLE_IP="192.168.139.1$(printf "%02d" "${WAFFLE_NO}")"

# Check the content of robot_mode file and set ENV VARS accordingly
if [ ! -f $HOME/.diamond/robot_mode ]; then
    echo "robot" > $HOME/.diamond/robot_mode
fi 

if grep -qi "robot" $HOME/.diamond/robot_mode; then
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID=$WAFFLE_NO
    MODE="robot"    
elif grep -qi "sim" $HOME/.diamond/robot_mode; then
    unset RMW_IMPLEMENTATION
    export ROS_DOMAIN_ID=1
    MODE="sim"
else
    echo "Unsupported robot mode set in file '${HOME}/.diamond/robot_mode'."
    echo "Please set it to 'robot' or 'sim' using the robot_mode command."
    MODE="unknown"
fi

export ZENOH_CONFIG_OVERRIDE="mode='client';connect/endpoints=['tcp/${WAFFLE_IP}:7447']" 

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

ros2() {
    # check to see if zenoh is running before running ros2 commands
    # (and then do what exactly...?)
    if [[ "$MODE" == "robot" ]]; then
        if ! pgrep -x "rmw_zenohd" > /dev/null; then
            echo "Zenoh Middleware doesn't appear to be running."
            echo "This must be running in order to establish/maintain a connection with your robot."
            echo "Run the following command to launch it:"
            echo " > ros2 run rmw_zenoh_cpp rmw_zenohd"
            return 1
        fi
    fi
    command ros2 "$@"
}
