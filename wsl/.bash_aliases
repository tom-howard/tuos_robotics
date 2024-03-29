# General
alias wxp="echo 'Opening the current directory in Windows Explorer...' && explorer.exe ."
alias off="echo 'Shutting down WSL...' && wsl.exe --shutdown"
alias src="echo 'Sourcing bashrc...' && source ~/.bashrc"

# TB3
alias tb3_teleop="rosrun turtlebot3_teleop turtlebot3_teleop_key"
alias tb3_world="roslaunch turtlebot3_gazebo turtlebot3_world.launch"
alias tb3_empty_world="roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch"
alias tb3_slam="roslaunch turtlebot3_slam turtlebot3_slam.launch"
alias tb3_rviz="roslaunch tuos_simulations rviz.launch"

# MiRo
alias miro_sim='cd ~/mdk/sim && ./launch_sim.sh'
alias miro_full='cd ~/mdk/sim && ./launch_full.sh'
alias miro_gui='cd ~/mdk/bin/shared && ./client_gui.py'
alias cd_mdk='cd ~/mdk/catkin_ws/src/'
