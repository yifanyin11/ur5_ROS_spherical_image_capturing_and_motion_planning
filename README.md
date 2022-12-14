# UR5-ROS Spherical Image Capturing and Motion Planning

This ROS package provides modules for sending a manipulator (UR5) to target objects (registered using AR tags), and performing image captures around them in multiple layers of spherical surfaces. Two versions of scanning processes are implemented. Users are able to perform either an automatic scanning (implemented by _MoveIt!_ movegroup interface), or a semi-automatic scanning (implemented by Rviz motion planning plugin).

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

* Launch the environment

To activate the software setups, run

```
roslaunch vision_guided_planning vision_guided_planning.launch
```
It will start the robot driver, rviz, camera driver and activate all other necessary settings.

<p>&nbsp;</p>

* Register two objects to tf tranform tree using Aruco markers

To register object1, move the end effector around, so that the marker can be viewed in the camera mounted. Open a new terminal, run
```
roslaunch vision_guided_planning register_object1.launch
```
A message will show up once if the transformation has been sent. Repeat the same procedure for object2,
```
roslaunch vision_guided_planning register_object2.launch
```

<p>&nbsp;</p>

* Perform spherical image capturing

To start the automatic version of image capturing, run
```
roslaunch vision_guided_planning spherical_capturing.launch
```
It will capture images for both objects registered, as if the camera is moving on a sphere and facing towards the object at all time.

If rviz motion planning plugin is preferred, run
```
roslaunch vision_guided_planning spherical_capturing_rviz.launch
```

## Methods

The core of spherical image capturing is the way of determining rigid body transformations of the camera link with respect to the object. 

<p align="center">
  <img src="https://github.com/yifanyin11/ur5_ROS_spherical_image_capturing_and_motion_planning/blob/main/repo_data/method_1.png" width="355" height="300" />
</p>

As shown in the figure, the origin of the object frame is at the center of an imaginary sphere. Our goal is to calculate the transformation $E_{oc}$ (the inverse transform of the one noted in the figure), such that the camera is moving on the sphere, and facing towards the object at all time.

### Assumptions

In order to interpret this goal, we make the following assumptions:
* In the camera frame, an initial pose has been given so that the camera is facing towards the object. The translation component of that initial pose, in the view of the camera frame, is $\vec{p_0}$.
* When performing the spherical scanning, the translation component of the rigid body transformation $E_{co}$ is always equal to its initial value, only the rotation changes. That is 

<p align="center">
  $E_{co}=g(R_{co}, \vec{p_0})$.
</p>

### Spherical coordinate system

From the assumptions, we can write the expression for the transformation $E_{oc}$ as

<p align="center">
  $E_{oc}=g^{-1}\left(R_{co}, \vec{p}_0\right)=g\left(R^{T}_{co},\vec{p_r}\right)=g\left(R_{oc},\vec{p_r}\right)$,
</p>

where

<p align="center">
  $\vec{p_r}=-R_{oc}\vec{p_0}$.
</p>

In the object frame, the camera is on the sphere that centered at its origin. Thus, the origin of the camera frame in the view of the object, denoted as $\vec{p_r}$, can be determined in terms of the variables in spherical coordinate system as

<p align="center">
  $\vec{p}_r=\vec{f}(||\vec{p_0}||, \ \theta, \  \varphi)=[\ ||\vec{p_0}||\sin \theta \cos \varphi \quad ||\vec{p_0}||\sin \theta \sin \varphi \quad ||\vec{p_0}||\cos \theta\ ]^T$,
</p>

with $0 \leqslant \theta \leqslant \pi$ and $0 \leqslant \varphi \leqslant 2 \pi$.

### Active transformation 

Use exponential coordinates of $SO(3)$, set

<p align="center">
  $e^{\hat{\omega}\alpha}=R_{oc}$
</p>

<p align="center">
  $\vec{p_t}=-\vec{p_r}$
</p>

which yields

<p align="center">
  $e^{\hat{\omega}\alpha} \ \vec{p_0}=\vec{p_t}$
</p>

By active transformation interpretation, the equation above means $\vec{p_0}$ rotates about axis $\vec{\omega}$ by angle of $\alpha$, ends up with $\vec{p_t}$.

There are multiple ways to solve the rotation $R_{oc}$. One way is given as follows:

<p align="center">
  <img src="https://github.com/yifanyin11/ur5_ROS_spherical_image_capturing_and_motion_planning/blob/main/repo_data/method_2.png" width="235" height="200" />
</p>

As shown in the figure, take $\alpha$ as the angle between the concurrent vectors $\vec{p_t}$ and $\vec{p_0}$, the rotational axis will be orthogonal to the plane formed by the two vectors, which can be calculated by cross product as

<p align="center">
  $\vec{\omega} = \frac{\vec{p_o}\times \vec{p_t}}{||\vec{p_0}\times \vec{p_t}||} = \frac{\vec{p_o}\times (-\vec{p_r})}{||\vec{p_0}\times (-\vec{p_r})||}$
</p>

The angle $\alpha$ is given by 

<p align="center">
  $\alpha = \cos^{-1} (\frac{\vec{p_0}\cdot \vec{p_t}}{||\vec{p_0}|| \cdot ||\vec{p_t}||}) = \cos^{-1} (\frac{\vec{p_0}\cdot (-\vec{p_r})}{||\vec{p_0}||^2}) $.
</p>

Thus, one solution will be 

<p align="center">
  $R_{oc}=e^{\hat{\omega}\alpha}$.
</p>

### Eigen vectors

Now, discuss other solutions. For a rotation $R_{oc}$, if the frame associated with it rotates about $\vec{p_0}$ by an angle of $\beta$, the resulting orientation will be $R_{oc} \ e^{\hat{p_0}\beta}$. Notice that $\vec{p_0}$ is an eigen vector of $e^{\hat{p_0} \beta}$ with an eigen value $\lambda =1$. Thus,

<p align="center">
  $R_{oc} \ e^{\hat{p_0}\beta} \ \vec{p_0} = R_{oc} \ \vec{p_0} = -\vec{p_r}$.
</p>

That means, for any $R_{oc}$ satisfies $R_{oc} \ \vec{p_0} = -\vec{p_r}$, $R_{oc} \ e^{\hat{p_0}\beta}$ is also a solution, which corresponds to an extra rotation about $\vec{p_0}$ by an angle of $\beta$. If $\beta=0$, it reduces to the solution obtained by active transformation.

### Solutions

To conclude, the rigid body transformation $E_{oc}$ is given by 

<p align="center">
  $E_{oc}=g\left(R_{oc},\vec{p_r}\right)=g\left(e^{\hat{\omega}\alpha} e^{\hat{p_0}\beta},\vec{p_r}\right)$,
</p>

where 

<p align="center">
  $\vec{p}_r=[\ ||\vec{p_0}||\sin \theta \cos \varphi \quad ||\vec{p_0}||\sin \theta \sin \varphi \quad ||\vec{p_0}||\cos \theta\ ]^T$,
</p>

<p align="center">
  $\vec{\omega} = \frac{\vec{p_0}\times (-\vec{p_r})}{||\vec{p_0}\times (-\vec{p_r})||}$,
</p>

<p align="center">
  $\alpha = \cos^{-1} (\frac{\vec{p_0}\cdot (-\vec{p_r})}{||\vec{p_0}||^2}) $.
</p>


The parameters are $\theta$, $\varphi$ and $\beta$. $\vec{p_0}$ is given by the initial condition.


## Authors

Yifan Yin

Email: [yyin34@jhu.edu](yyin34@jhu.edu)
