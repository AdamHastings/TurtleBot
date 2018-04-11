Turtlebot Lab Tools
===================

Packages for facilitating interfacing with the Turtlebot. The Turtlebot is controlled from a **commander** machine, which communicates with the robot over a ROS network.

## Dependencies ##
Ensure that each machine has the appropriate software and hardware setup.

*Note*: Guides refer to using ROS Indigo and Ubuntu 14.04, but the same steps apply if you are using ROS Kinetic and Ubuntu 16.04.

### Commander Machine ###
- [software setup](http://wiki.ros.org/turtlebot/Tutorials/indigo/PC%20Installation)
- In terms of hardware, you will want to use the Logitech EXTREME 3D PRO controller with the turtlebot_commander package to drive the turtlebot. Or, alternatively, you could create your own node to interface with any teleop method of your choosing, using the package code as a template. The same topic the node publishes to, **/turtlebot_teleop_joystick/cmd_vel** (geometry_msgs::Twist), can be used for autonomous driving applications with the Turtlebot.

### Turtlebot Machine ###
- [software setup](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation)
- [hardware setup](http://learn.turtlebot.com/2015/02/01/3/)

## Using the Packages to Control the Turtlebot ##

Clone this repository into a catkin workspace on both the commander and Turtlebot machines. Ensure that both machines are on the same ROS network.

On the Turtlebot machine, run:
```
roslaunch turtlebot_recorder stream.launch
```
to stream the following topics over the network:
- **/camera/rgb/camera_info** (camera distortion parameters)
- **/camera/rgb/image_raw** (camera image stream)
- **/odom** (odometry readings)
- **/mobile_base/commands/velocity** (velocity commands)

or, alternatively:
```
roslaunch turtlebot_recorder record.launch
```
to record the topics to a rosbag in ~/.ros/.

On the commander machine, run:
```
roslaunch turtlebot_commander logitech_control.launch
```
to control the Turtlebot with the Logitech controller mentioned in the Dependencies section.

## Robotic Vision Students ##
On the Turtlebot, run;
```
roslaunch turtlebot_recorder stream.launch
```
On the commanding maching, run:
```
roslaunch turtlebot_cv opencv_control.launch
```

Additionally, on the commanding machine you need to set these local variables:
- ROS_MASTER_URI=http://(turtlebot's IP address):11311
- ROS_HOSTNAME=(commanding machine ip address)
- ROS_IP=(commanding machine ip address)

Your code should only need to change the `image_callback` function in the python script:
```
lab_turtlebot/turtlbot_cv/src/cv_command.py
```
Here you have an OpenCV image `cv_image` and a way to command forward velocity `v` and angular velocity `w` of the turtlebot with `send_command(v, w)`.


## Functionality Expansion ##
See additional launch files and packages in this repository for examples of other students' efforts in integrating additional sensors (such as LIDAR) with the Turtlebot.
