# ROS2-CPP-Turtlebot-Project

A ROS2 Project to implement the Basics of ROS2 in C++ <br>
Integration of a Custom Service Message <br>
Integration of Service Server to find the nearest wall <br>
Integration of a Service Client to call the Service Server <br>
Integration of a Custom Action Message <br>
Integration of a Action Server to record the Odometry of the robot <br>
Integration of a Action Client to start the recording of the Odometry <br>
A script which makes the robot to follow the wall on the right side for a specific distance <br>
A P-Controller which adjusts the angular velocity of the robot <br>

The Script has been successfully tested on the Turtlebot <br>
The Project was done in ROS2 Foxy <br>

Possible improvements:
Handle the case, when the robot would collide with the wall <br>
Example when the front_laser_value = 'inf', that the robot would then move backwards
