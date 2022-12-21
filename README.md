# UR5-ROS Spherical Image Capturing and Motion Planning

This ROS package provides modules for sending a manipulator (UR5) to target objects (registered using AR tags), and performing image captures around them in mutiple layers of spherical surfaces. Two versions of scanning processes are implemented. Users are able to perform either an automatic scanning (implemented by _MoveIt!_ movegroup interface), or a semi-automatic scanning (implemented by Rviz motion planning plugin). The robot motion planning algorithms used can be created either from built-in libraries or custom context.

## Uses
Any applications in which images of (an) object(s) from different angle of views need to be captured, especially when the depth of the objects needs to be specified or precisely controlled. Below are several examples:

1. Image collections for neural network training in classification, detection or segmentation applications
2. Experimental visual validations from all angle of views

## Getting Started

### Dependencies

The package runs on Ubuntu 18.04 and ROS Melodic.

### Installing
A Catkin workspace is required for the build of this package.
```
cd $HOME/catkin_ws/src

# retrieve the sources
git clone https://github.com/yifanyin11/ur5_ROS_spherical_image_capturing_and_motion_planning.git spherical_image_capturing/

# install dependencies from sources
git clone -b kinetic-devel https://github.com/ros-industrial/robotiq.git 
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git 
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git 
git clone -b old/indigo-devel https://github.com/IntelRealSense/realsense-ros.git
git clone -b melodic-devel https://github.com/pal-robotics/aruco_ros.git 
git clone -b v1.12.4 https://github.com/IntelRealSense/librealsense.git

# checking other dependencies
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -y

# building
catkin build

# activate this workspace
source devel/setup.bash
```

### Executing program

* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Methods

The core of this package is the way of determining rigid body transformations of the camera link in the frame of the object. 

<img src="https://github.com/yifanyin11/ur5_ROS_spherical_image_capturing_and_motion_planning/blob/main/repo_data/method1.png" width="200" height="200" />

## Authors

Contributors names and contact info

Yifan Yin

Email: [yyin34@jhu.edu](yyin34@jhu.edu)
