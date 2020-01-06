# README

This package was tested with:

- Ubuntu 16.04 LTS - Ubuntu 18.04 LTS
- ROS Kinetic + Melodic

Performances are slightly different between the two ROS versions. On Melodic, exploration is slightly faster for some reasons. Data on the paper are obtained using Kinetic.

Dependencies:

- turtlebot_gazebo
- rviz
- gazebo_ros
- gazebo_plugins
- turtlebot_description
- turtlebot_navigation
- nav2d_navigator
- nav2d_operator
- nodelet
- tf
- nav_msgs
- roscpp
- rospy
- std_msgs
- robot_state_publisher
- map_server
- kobuki
- kobuki_desktop

Set the robot model as variable in  .bashrc `export TURTLEBOT_BASE=kobuki` 

After you've built everything, to launch an exploration you have to do:

1. cd < path workspace >

2. catkin_make

3. source devel/setup.bash

4. roslaunch blueprint_exploration turtlebot_expl.launch

5. rosservice call /StartExploration

You can opt in/out the Gazebo GUI and RVIZ. There is one launchfile for each version of the world used. 
