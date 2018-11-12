## How to use this repository


### On a real husky

This repository has nodes which allow a husky to derive its position, and navigate to preselected waypoints.  It is organized around the navigation stack, built on the packages robot_localization (http://wiki.ros.org/robot_localization), move_base (http://wiki.ros.org/move_base), gmapping (http://wiki.ros.org/gmapping).

The robot localization nodes can be started with the launch file:

    roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch

The robot can then navigate to GPS coordinates by opening another terminal and running the command

    rosrun outdoor_waypoint_nav global_checkpoints <lat> <lon> <lat> <lon> ...

where a list of latitude and longitude coordinates can be input as pairs, separated by a space.  So far, the coordinates are input as command line arguments.  A file reader will be added.  


### In simulation

The software can be tested in Gazebo/Rviz with launch file:

    roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch

and the GPS navigation can be run as above.




