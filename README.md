haptic-ros-control
----------------

# Overview
This ROS Package is a wrapper for the Force Dimension SDK, allowing user to provide inputs from both Sigma.7 and Omega.7 haptics devices to ROS robots.  

# Setup Instructions
## Force Dimension SDK
1. Download the linux SDK from Force Dimension's [website](https://www.forcedimension.com/software/sdk)

2. Copy all files in 
``` sdk-3.14.0/lib/release/lin-x86_64-gcc ```
to 
```usr/local/lib ```

3. Copy all files in 
```sdk-3.14.0/include```
to
```usr/local/include ```

To use roslaunch, the haptics device must be added as super to allow USB communication. See [this ROS answer](https://answers.ros.org/question/219395/ros-node-does-not-recognize-haptic-device/) for details on how to add device as super. 

## ROS Package

clone and build the package from src

	cd ~/catkin_ws/src
	git clone https://github.com/SchultzKyle/haptics-ros-control.git
	cd ..
	catkin_make

# Usage

<ol>
  <li>sigma_vel_cmd.cpp and sigma_cmd_vel.launch</li>
    	- Converts Sigma haptic device inputs into linear velocity topic<br>  
  <li>omega_vel_cmd.cpp and omega_cmd_vel.launch</li>
    	- Converts Omega haptic device inputs into linear velocity topic<br>  
</ol>
