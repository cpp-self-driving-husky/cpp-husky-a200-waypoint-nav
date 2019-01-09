## How to use this repository


### Introduction

This repository has nodes which allow a husky to derive its position, and navigate to preselected waypoints.  It is organized around the navigation stack, built on the packages robot_localization (http://wiki.ros.org/robot_localization), move_base (http://wiki.ros.org/move_base), gmapping (http://wiki.ros.org/gmapping).  It is strongly recommended that anyone who wishes to use robot localization should thoroughly read the tutorial for robot_localization at http://docs.ros.org/indigo/api/robot_localization/html/index.html, especially paying attention to the parameter descriptions of the navsat_transform_node page.


### Running on Husky

The robot localization nodes can be started with the launch file:

    roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch

The robot can then navigate to GPS coordinates by opening another terminal and running the global_checkpoints node with the command

    rosrun outdoor_waypoint_nav global_checkpoints <lat> <lon> <lat> <lon> ...

where a list of latitude and longitude coordinates can be input as pairs with each number separated by a space.  


#### outdoor_waypoint_nav launch file

The launch command above refers to the file <repo>/outdoor_waypoint_nav/launch/outdoor/outdoor_waypoint_nav.launch.  This assumes the bring-up nodes and sensor drivers have not been started (on our husky they have, but not with the right parameters, which is discussed in the startup notes section).  The launch files which this file starts can be found throughout this repo, including the sensors, the navsat_transform and robot_localization (as in the localization_run.launch file), gmapping, and move_base.  Also, importantly, teleoperation nodes are launch which allow a user to steer the husky with a controller should the husky veer too far off course.

The gmapping local map is somewhat small at this point to save processing power. This can be increased by altering the necessary parameters. There is some degree of obstacle avoidance in gmapping, but it is not mission-ready.


#### global_checkpoints node

The coordinates are input as command line arguments, and the node exits once the final point is reached.  Of course, this will need to be updated once the user interface and the methods to deliver the coordinates are implemented, which can be done by revising the file <repo>/outdoor_waypoint_nav/src/global_checkpoints_node.cpp to continuously iterate.  The 'work' of this node is done in the PathGenerator class in the global_checkpoints.h file.  The robot moves in a straight line between the input points.


#### startup notes

The initial configuration files that launch upon startup of the husky (factory settings) are wrong for our configuration. To run the waypoint nav node on the husky properly, it is necessary to

	rosnode kill /navsat_transform /ekf_localization

then startup the launch file outdoor_waypoint_nav.launch, ctrl+c to kill it in the same window, and run it again.  This will reset and restart all of the necessary nodes with the correct parameters.


#### IMU

Unfortunately, the IMU on the husky has relatively unstable trial to trial measurements, where the values are not particularly consistent from one startup to another.  This could be improved with a better IMU, or some means of calibration.  The current way to calibrate it is with a rosservice

	rosservice call /imu/calibrate

and, in another window, check the success of the calibration with

	rostopic echo /imu/is_calibrated

which will give a boolean value as to whether or not the calibration worked.  It may require several attempts before a 'true' value is returned.  The outdoor_waypoint_nav.launch will still work, but results are improved by doing a hack calibration, which requires comparing the movement of the husky with the simulated movement of the husky in Rviz.  Rviz can be run on the husky by uncommenting the node under the <-- Run rviz --> and <-- Using custom configuration file --> comments at the bottom of the outdoor_waypoint_nav.launch file.  The next time the node is run, an interface with the Rviz sim is displayed on the local machine from which the commands are run.  The husky should be driven forward with a controller.  There may be some discrepancy between how the husky is actually moving via this controller, and how it is visualized in Rviz.  Of course, they should be the same.  To fix this, run the hack calibration node with

	roslaunch outdoor_waypoint_nav heading_calibration.launch

making sure that there are 5 meters of free space in front of the husky.  Then, restart the node by killing the outdoor_waypoint_nav nodes (press ctrl+c in the window where the launch file was launched) and repeating this process until the movement of the husky matches that of Rviz relatively well.


### In simulation

The software can be tested in Gazebo/Rviz with launch file:

    roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch

and the GPS navigation can be run as above, though the GPS coordinates in Gazebo refer to somewhere with a latitude around 49.9 and longitude around 8.9, nowhere near California.  Before running, the husky description urdf would have to be revised by commenting out the top_plate_link and top_plate_joint between the <link allows for connection of top plate> comments found in the file at  <repo>/husky_customization/husky_custom_gazebo/urdf/custom_description.urdf.xacro.  This should only be done when running simulation, not on the husky itself due to a discrepancy between the model for the husky in local ROS installations and the one on the husky itself.
