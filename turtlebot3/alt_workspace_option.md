Instead of having a ROS2 workspace for each user ("robot" and "waffle"), this seems to work instead:

(As user "Waffle"):

Create a new dir in `/home/

```
sudo mkdir -p /home/ros/
```

Create a new group called `rosgrp` and add users to it:

```
sudo addgroup rosgrp
sudo adduser waffle rosgrp
sudo adduser robot rosgrp
```

Change ownership of `/home/ros/` to `waffle` and change its group to `rosgrp`:

```
sudo chown waffle:rosgrp /home/ros/
```

Make a workspace:

```
mkdir -p /home/ros/tb3_ws/src
```

and so on...

... It seems that if both users source the `home/ros/tb3_ws/install/local_setup.bash` file then turtlebot3_bringup works...