# General
alias src="echo 'Sourcing the .bashrc...' && source ~/.bashrc"

# some useful TurtleBot3 commands:
alias tb3_teleop="roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
alias tb3_world="roslaunch turtlebot3_gazebo turtlebot3_world.launch"
alias tb3_empty_world="roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch"
alias tb3_bringup="roslaunch turtlebot3_bringup turtlebot3_remote.launch"
alias tb3_slam="roslaunch turtlebot3_slam turtlebot3_slam.launch"
alias tb3_rviz="rosrun rviz rviz -d \`rospack find turtlebot3_description\`/rviz/model.rviz"
alias cd_cat="cd ~/catkin_ws/src"

# some useful MiRo commands:
alias miro_sim='cd ~/mdk/sim && ./launch_sim.sh'
alias miro_full='cd ~/mdk/sim && ./launch_full.sh'
alias miro_gui='cd ~/mdk/bin/shared && ./client_gui.py'
alias cd_mdk="cd ~/mdk/catkin_ws/src"
