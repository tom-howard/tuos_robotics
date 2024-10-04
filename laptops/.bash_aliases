# utility
alias src="echo 'Sourcing ~/.bashrc...' && source ~/.bashrc"
alias ros_cd="cd ~/ros2_ws/src"

# some useful TurtleBot3 commands:
alias tb3_teleop="ros2 run turtlebot3_teleop teleop_keyboard"
alias tb3_cartographer="ros2 launch turtlebot3_cartographer cartographer.launch.py"
alias tb3_rviz="ros2 launch turtlebot3_bringup rviz.launch.py"
alias tb3_world="ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
alias tb3_sim="ros2 launch turtlebot3_gazebo empty_world.launch.py"

alias rsd="ros2 daemon stop; ros2 daemon start;"
