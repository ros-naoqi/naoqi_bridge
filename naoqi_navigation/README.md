# Navigation RVIZ

## Overview

Some tools are available for visualization of the features provided by ALNavigation and ALRecharge modules.

These tools use the ROS Visualization framework rviz (http://wiki.ros.org/rviz).

This documentation will focus on setup and use on a Ubuntu 14.04 machine using ros-indigo distribution.

## Installation

### Ros core installation

First, we need to install ros core framework on the machine, because rviz uses ros middleware for message transfer.

All instructions are provided here: http://wiki.ros.org/indigo/Installation/Ubuntu.

Basically, you will need to:

    sudo apt-get install ros-indigo-base

### Naoqi packages

You will now install Naoqi packages for rviz. Some of them are distributed through Ubuntu package manager, some other are Aldebaran private and will need to be built manually.

#### Prebuilt Packages

    # install rviz
    sudo apt-get install ros-indigo-rviz ros-indigo-naoqi-bridge
    ros-indigo-naoqi-bridge-msgs ros-indigo-pepper-robot
    ros-indigo-pepper-description ros-indigo-pepper-meshes
    ros-indigo-octomap-msgs

#### Custom build packages

To build the navigation-specific packages, you will need to use catkin.

    sudo apt-get install ros-indigo-catkin

You will then need to create a catkin worktreea and clone the naoqi_bridge repo.

    source /opt/ros/indigo/setup.bash
    cd ~ && mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
    catkin_init_workspace
    git clone https://github.com/ros-naoqi/naoqi_bridge.git
    cd ..
    catkin_make

Now you should have all what is needed to visualize navigation features in Rviz.

## Running Rviz

Each time, you want to use those features, you will need to source two files to find the correct libs.

    source /opt/ros/indigo/setup.zsh && source ~/catkin_ws/devel/setup.zsh

Alternatively, you can add an alias in you .bashrc to source them in one command.

To connect to a robot a display the debug, you need to run:

    roslaunch naoqi_navigation navigation_full_rviz.launch nao_ip:=ip_of_your_robot

This will start all navigation nodes, pepper display nodes and a rviz instance.

In rviz set fixed frame to **map**.

You can display navigation nodes by loading the config (in rviz "File" -> "Open config") located in ~/catkin_ws/src/naoqi_bridge/naoqi_navigation/python/navigation_config.rviz

*Note*

If you have problem with robot meshes check if folder **/opt/ros/indigo/share/pepper_meshes/meshes/1.0/**
exists and contain all meshes.

You can use this page (http://wiki.ros.org/rviz/UserGuide) to learn how to use rviz and display nodes to the current view.
