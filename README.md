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

The core of spherical image capturing is the way of determining rigid body transformations of the camera link with respect to the object. 

<p align="center">
  <img src="https://github.com/yifanyin11/ur5_ROS_spherical_image_capturing_and_motion_planning/blob/main/repo_data/method1.png" width="475" height="400" />
</p>

As shown in the figure, the origin of the object frame is at the center of an imaginary sphere. Our goal is to calculate the transformation $E_{OC}$, such that the camera is moving on the sphere, and facing towards the object at all time.

### Assumptions

In order to interpreate this goal, we make the following assumptions:
* In the camera frame, an initial pose has been given so that the camera is facing towards the object. The translation component of that initial pose, in the view of the camera frame, is $p_0$.
* When performing the spherical scanning, the translation component of the rigid body transformation $E_{CO}$ is always equal to its initial value, only the rotation changes. That is 

<p align="center">
  $E_{CO}=g(R, \vec{p_0})$
</p>

### Spherical coordinate system

From the assumptions, we can write the expression for the transformation $E_{OC}$ as

<p align="center">
  $E_{OC}=g^{-1}\left(R, \vec{p}_0\right)=g\left(R^{T},-R^{T} \vec{p}_0\right)$
</p>

where

<p align="center">
  $\vec{p_r}=-R^{T}\vec{p_0}$
</p>

In the object frame, the camera is on the sphere that centered at its origin. Thus, the origin of the camera frame in the view of the object, denoted as $\vec{p_r}$, can be determined in terms of the variables in spherical coordinate system as

<p align="center">
  $\vec{p}_r=\vec{f}(||\vec{p_0}||, \theta, \varphi)=[||\vec{p_0}||\sin \theta \cos \varphi, ||\vec{p_0}||\sin \theta \sin \varphi, ||\vec{p_0}||\cos \theta]^T$
</p>

## Authors

Contributors names and contact info

Yifan Yin

Email: [yyin34@jhu.edu](yyin34@jhu.edu)
