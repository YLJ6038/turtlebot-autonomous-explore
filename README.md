# Frontier-based-explore
Frontier-based explore package for turtlebot(Kinect) simulation and real-world application

This is the final project for EN 530.707 Robot System Programming course in Johns Hopkins University. In this project, we want to develop several kinds of algorithms and packages in ROS to control the turtlebot( with Kinect) to autonomously explore the environment and build the map. With the map, we can navigate turtlebot from one starting point to another desire point.

The basic information about turtlebot in ROS can be found here: [Turtlebot](http://wiki.ros.org/Robots/TurtleBot)

How to set up, configure and bring up a Turtlebot： [Turtlebot Tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo)

### The ROS Packages we used for this project:

[Turtlebot_gazebo](https://github.com/turtlebot/turtlebot_simulator): Turtlebot simulation 

[Turtlebot_navigation](https://github.com/turtlebot/turtlebot_apps): AMCL, gmapping and Kinect

[Octomap](https://github.com/OctoMap/octomap) : 3D mapping and visualization 

And if you'd like to know more about this project: [The presentation PPT can be found here](https://docs.google.com/presentation/d/1sejKV5Q7UtgZKAZdH1bs6nY4dMpWcq0Tav3_yq_jBV8/edit#slide=id.gd749d66d51_0_130)

For any questions, you can communicate with: Longji Yin(ljyin6038@gmail.com) and Ruixin Li(ruixinlee424@gmail.com)

## The Structure of the Package:

The structure and working process of the Frontier-based explore package is shown in this activity diagram:

<img src="https://github.com/YLJ6038/frontier-based-explore/blob/master/Figures/activity_map_frontier_explore.png" width="700" />

## ROS API Documentation:

**[Documentation for frontier explore package can be found here](https://github.com/YLJ6038/frontier-based-explore/blob/master/Docs/ros_api_frontier_explore_pkg.md)**

## How to run this package:

### Install Dependencies:

```
sudo apt-get install ros-kinetic-turtlebot*
sudo apt-get install ros-kinetic-octomap*
```

### Run Simulation

If you want to run this package in simulation, you should run follow commands in different terminal windows:

```
roslaunch frontier_explore gazebo.launch
roslaunch frontier_explore preload.launch
rosrun frontier_explore explore.launch
```

You should be able to see the simualtion process in RVIZ and Gazebo. The simulation result can be seen from the pictures below:

<div align=center><img width="800" height="600" src="https://github.com/YLJ6038/frontier-based-explore/blob/master/Figures/simulation_gazebo_map_frontier_explore.png"/></div>

<div align=center><img width="800" height="600" src="https://github.com/YLJ6038/frontier-based-explore/blob/master/Figures/gazebo_3d_map_frontier_explore.png"/></div>

### Run real-world Turtlebot

If you use one laptop to connect with turtlebot and one remote pc to control the turtlebot, please follow the [turtlebot network configuration](http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration) to setup first.

Then from turtlebot, run: 

```
roslaunch turtlebot_bringup minimal.launch
roslaunch frontier_explore preload_realworld.launch
rosrun frontier_explore frontier_explore_node
```

From remote pc. run:

```
roslaunch turtlebot_exploration_3d exploration_rviz.launch
```

# Information-Theoretic Exploration

In this package, we develop an ROS implementation of infomation-theoretic exploration using turtlebot with Kinect. This package is based on the [turtlebot_exploration_3d](https://github.com/RobustFieldAutonomyLab/turtlebot_exploration_3d) package. We modify the package and apply it into our turtlebot and project.

## How to run this package:

### Run Simulation

If you want to run this package in simulation, you should run follow commands in different terminal windows:

```
roslaunch frontier_explore gazebo.launch
roslaunch frontier_explore preload.launch
rosrun info_explore info_explore_node
```

You should be able to see the simualtion process in RVIZ and Gazebo. The simulation result can be seen from the pictures below:

<div align=center><img width="800" height="400" src="https://github.com/YLJ6038/frontier-based-explore/blob/master/Figures/information_theory_simulation.png"/></div>

## Reference

- S. Bai, J. Wang, F. Chen, and B. Englot, "Information-Theoretic Exploration with Bayesian Optimization", IEEE/RSJ International Conference on Intelligent Robots and Systems(IROS), October 2016.

- Yamauchi, Brian, "Frontier-based exploration using multiple robots", Proceedings of the second international conference on Autonomous agents, 1998
