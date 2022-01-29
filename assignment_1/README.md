## Instructions to run the code

Ensure doing the following before running the code.

```shell
cd ~/catkin_ws/src
git clone https://github.com/SDeepakMallya/sc627_helper.git
git clone https://github.com/TheSmilingSky/sc627_assignments.git
cd ..
catkin_make
source devel/setup.bash
cd src/sc627_assignments/assignment_1
chmod +x bug_1.py
chmod +x bug_base.py
cd ../../sc627_helper
chmod +x move_xy_server.py
``` 

In four parallel terminals, run the following to visualize the bug 1 simulation.

```shell
roscore
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
rosrun sc627_helper move_xy_server.py
rosrun sc627_assignments bug_1.py
```

The generated results post simulations will be stored in the 'data/Bug1' folder inside the assignment_1 folder.

A generated result visualization:

![Path viz](savedData/Bug1/path.png)
