# Turtlebot-autonomous-explore
Autonomous explore packages for turtlebot(Kinect) simulation and real-world application

This is the final project for EN 530.707 Robot System Programming course in Johns Hopkins University. In this project, we developed two ROS packages to control the turtlebot(with Kinect) to autonomously explore the environment and build the map. We implemented two different exploration algorithms to achieve the task: frontier-based exploration and information-theoretic exploration.

The basic information about turtlebot in ROS can be found here: [Turtlebot](http://wiki.ros.org/Robots/TurtleBot)

How to set up, configure and bring up a Turtlebot： [Turtlebot Tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo)

### The ROS Packages we used for this project:

[Turtlebot_gazebo](https://github.com/turtlebot/turtlebot_simulator): Turtlebot simulation 

[Turtlebot_navigation](https://github.com/turtlebot/turtlebot_apps): AMCL, gmapping and move_base

[Octomap](https://github.com/OctoMap/octomap) : 3D mapping and visualization 

The presentation PPT can be found [here](https://docs.google.com/presentation/d/1sejKV5Q7UtgZKAZdH1bs6nY4dMpWcq0Tav3_yq_jBV8/edit#slide=id.gd749d66d51_0_130)

For any questions, please contact: Longji Yin(ljyin6038@gmail.com) and Ruixin Li(ruixinlee424@gmail.com)

# Frontier-based-explore package

This package provides frontier-based exploration for turtlebot. While running the package, turtlebot will explore the unknown environment until no valid frontier can be detected. The package mainly contains two nodes: frontier_explore_node is for exploration process control, detect_frontiers_server is for computational analysis of the map. 

## ROS API Documentation:

**[Documentation for frontier explore package can be found here](https://github.com/YLJ6038/frontier-based-explore/blob/master/Docs/ros_api_frontier_explore_pkg.md)**

## Activity diagram of the Package:

The architecture and working process of the Frontier-based explore package is shown in this activity diagram:

<img src="https://github.com/YLJ6038/frontier-based-explore/blob/master/Figures/activity_map_frontier_explore.png" width="750" />



## How to run this package:

### Install Dependencies:

```
sudo apt-get install ros-kinetic-turtlebot*
sudo apt-get install ros-kinetic-octomap*
```

### Download from source:

```
my_catkin_workspace/src$ git clone 
my_catkin_workspace/src$ cd ..
my_catkin_workspace$ catkin build
```

### Run Simulation:

If you want to run this package in simulation, you should run follow commands in different terminal windows:

```
roslaunch frontier_explore gazebo.launch
roslaunch frontier_explore preload.launch
roslaunch frontier_explore explore.launch
```

You should be able to see the simualtion process in RVIZ and Gazebo. The simulation result can be seen from the pictures below:

<div align=center><img width="500" height="375" src="https://github.com/YLJ6038/frontier-based-explore/blob/master/Figures/simulation_gazebo_map_frontier_explore.png"/></div>

<div align=center><img width="500" height="375" src="https://github.com/YLJ6038/frontier-based-explore/blob/master/Figures/gazebo_3d_map_frontier_explore.png"/></div>

### Run real-world Turtlebot

If you use one laptop to connect with turtlebot and one remote pc to control the turtlebot, please follow the [turtlebot network configuration](http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration) to setup first.

Then from the turtlebot, run: 

```
roslaunch turtlebot_bringup minimal.launch
roslaunch frontier_explore preload_realworld.launch
rosrun frontier_explore frontier_explore_node
```

From remote pc. run:

```
roslaunch frontier_explore exploration_rviz.launch
```

# Information-theoretic exploration package

This package is a ROS implementation of information-theoretic exploration for turtlebot with Kinect. The core implementation is based on the [turtlebot_exploration_3d](https://github.com/RobustFieldAutonomyLab/turtlebot_exploration_3d) package. We rearranged the code structure and defined two new classes for exploration process control and computation.
We modified the interfaces and make the package fit into our ros distro and turtlebot.

## ROS API Documentation:

**[Documentation for information-theoretic explore package can be found here](https://github.com/YLJ6038/turtlebot-autonomous-explore/blob/master/Docs/ros_api_info_explore_pkg.md)**

## How to run this package:

### Install Dependencies:

```
sudo apt-get install ros-kinetic-turtlebot*
sudo apt-get install ros-kinetic-octomap*
```

### Download from source:

```
my_catkin_workspace/src$ git clone 
my_catkin_workspace/src$ cd ..
my_catkin_workspace$ catkin build
```

### Run Simulation

If you want to run this package in simulation, you should run follow commands in different terminal windows:

```
roslaunch info_explore gazebo.launch
roslaunch info_explore preload.launch
rosrun info_explore info_explore_node
```

You should be able to see the simualtion process in RVIZ and Gazebo. The simulation result can be seen from the pictures below:

<div align=center><img width="700" height="350" src="https://github.com/YLJ6038/frontier-based-explore/blob/master/Figures/information_theory_simulation.png"/></div>

## Reference

- S. Bai, J. Wang, F. Chen, and B. Englot, "Information-Theoretic Exploration with Bayesian Optimization", IEEE/RSJ International Conference on Intelligent Robots and Systems(IROS), October 2016.

- Yamauchi, Brian, "Frontier-based exploration using multiple robots", Proceedings of the second international conference on Autonomous agents, 1998
