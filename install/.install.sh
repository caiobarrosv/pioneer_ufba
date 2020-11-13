#!/bin/bash
####################################
#
# Install the necessary packages 
#
####################################

python -m pip install -U pip setuptools
python -m pip install matplotlib
python -m pip install django
python -m pip install pandas

echo '###### Installing some controllers ######'
# Install catkin tools
sudo apt-get install ros-kinetic-catkin python-catkin-tools

# Install some controllers
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-position-controllers
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

sudo apt-get install ros-kinetic-navigation 
sudo apt-get install ros-kinetic-slam-gmapping 
sudo apt-get install ros-kinetic-ros-control 
sudo apt-get install ros-kinetic-ros-controllers 
sudo apt-get install ros-kinetic-rviz
sudo apt-get install ros-kinetic-moveit-ros-visualization

echo '###### Installing and upgrading pip | pathlib ######'
apt install python-pip
pip install --upgrade pip
pip install pathlib
