# FSR_PROJ
The project has been entirely implemented in ROS Melodic and Gazebo 9 as a final course project for a Field and Service Robotics course atUniversity Federico II of Naples.
The environment has been derived from Amazon Web Service RoboMaker github folder and has been modified for the specific project.
# To install:
```
$ cd <catkin_ws>/src
$ git clone https://github.com/AndCav/FSR_PROJ.git
$ cd ..
$ catkin_make
```

# To use:
```
To make sure your workspace is properly overlayed by the setup script:
$ source <catkin_ws>/devel/setup.sh
to launch the system: 
$ roslaunch mapper bookstore_tb3.launch ( add gui:=false if the gui is not needed)
$ roslaunch mapper rviz.launch (this one is not mandatory, available to open a related rviz window)

```
# Necessary dependencies :

```
$ sudo apt-get install ros-melodic-map-server

--probably already downloaded dependencies but needed--
gazebo
gazebo_ros
gazebo_ros_control
gazebo_ros_pkgs
joint_state_controller
position_controllers
velocity_controllers
robot_state_publisher
ros_controllers

```

# Note:
```
The folder contains also demonstration videos and a detailed report.
```
