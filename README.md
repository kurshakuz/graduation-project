# autonomous-skid-steering-robot
Autonomous Skid-Steering Based Mobile Robot-Manipulation System for Automating Warehouse Operations

## Software Requirements for Social Navigation Part
* ROS installation
* Ubuntu

## Instalation instructions
This set of instructions was tested for Ubuntu18 with ROS-Melodic. Additionally, we assume that you already have a complete ROS installation.
* Please follow the following instructions:
```
sudo apt-get install ros-melodic-jackal-control ros-melodic-jackal-gazebo ros-melodic-jackal-simulator ros-melodic-jackal-description ros-melodic-jackal-desktop ros-melodic-jackal-navigation ros-melodic-jackal-viz
sudo apt install ros-melodic-people-msgs
cd [workspace]/src
git clone https://github.com/ALARIS-NU/social_navigation.git
cd pedsim_ros/
rosdep install --from-paths src --ignore-src -r -y
cd ../
catkin_make
source devel/setup.bash
```

## Running LMPCC
* Simulation Environment

        1. Start Jackal Gazebo Simulation
            * roslaunch jackal_gazebo jackal_world.launch
        2. Start Pedestrian Simulator
            * roslaunch pedsim_simulator corridor.launch
* Start lmpcc_obstacle_feed

        1. roslaunch lmpcc_obstacle_feed lmpcc_obstacle_feed.launch

* Start lmpcc controller

        1. roslaunch lmpcc lmpcc.launch

* Start rqt_reconfigure

        1. rosrun rqt_reconfigure rqt_reconfigure
        2. Click on the lmpcc parameters to start the robot motion by press the enable_output button
        3. Click on the lmpcc parameters to start planning by pressing the plan button