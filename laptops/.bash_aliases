# utility
alias src="echo 'Sourcing ~/.bashrc...' && source ~/.bashrc"
alias cd_workspace="cd ~/tb3_ws/src"

# some useful TurtleBot3 commands:
alias tb3_teleop="ros2 run turtlebot3_teleop teleop_key"
alias tb3_bringup="ros2 launch turtlebot3_bringup robot.launch.py"
alias tb3_cartographer="ros2 launch turtlebot3_cartographer cartographer.launch.py"
alias tb3_rviz="roslaunch tuos_tb3_tools rviz.launch"
alias tb3_world="ros2 launch turtlebot3_gazebo turtlebot3_world.launch"
alias tb3_sim="ros2 launch turtlebot3_gazebo empty_world.launch"
