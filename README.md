# ROS2-CPP-Turtlebot-Project

A ROS2 Project to implement the Basics of ROS2 in C++ 
Integration of a Custom Service Message
Integration of Service Server to find the nearest wall
Integration of a Service Client to call the Service Server
Integration of a Custom Action Message
Integration of a Action Server to record the Odometry of the robot
Integration of a Action Client to start the recording of the Odometry
A script which makes the robot to follow the wall on the right side for a specific distance
A P-Controller which adjusts the angular velocity of the robot

The Script has been successfully tested on the Turtlebot. 
The Project was done in ROS2 Foxy

Possible improvements:
Handle the case, when the robot would collide with the wall. 
Example when the front_laser_value = 'inf', that the robot would then move backwards
