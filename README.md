# autonomous-skid-steering-robot
Autonomous Skid-Steering Based Mobile Robot-Manipulation System for Automating Warehouse Operations

## Software Requirements for Social Navigation Part
* ROS installation
* Ubuntu
* Eigen3 (Please install Eigen3 from the following link:https://eigen.tuxfamily.org/index.php?title=Main_Page)
* rqt_multiplot
## Installation instructions
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
cd rover_jackal/trajectory_rover/cfg/
chmod +x set_trajectory_rover.cfg
cd [workspace]
catkin_make
source devel/setup.bash
```
## Running MPC
* Simulation Environment

        1. roslaunch jackal_gazebo empty_world.launch
        2. roslaunch mocap_rover mocap_bridge_gazebo.launch

* Start NMPC and trajectory

        1. roslaunch nmpc_pc_rover nmpc_pc_gazebo.launch
        2. roslaunch trajectory_rover trajectory_rover.launch

* Start rqt_multiplot 

        1. rqt_multiplot (use included files)
        
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

## Run Gazebo and Pedsim based LMPCC navigation

* Gazebo simulation world
        1. roslaunch cpr_office_gazebo office_world_with_plugin.launch
* LMPCC Obstacle Feed and Controller
        1. roslaunch lmpcc_obstacle_feed lmpcc_obstacle_feed.launch
        2. roslaunch lmpcc lmpcc.launch
* Pedsim people spawn and vizualization in rviz
        1. roslaunch pedsim_swarm_simulation pedsim_office_populated.launch
* Control RQT tool
        1. rosrun rqt_reconfigure rqt_reconfigure