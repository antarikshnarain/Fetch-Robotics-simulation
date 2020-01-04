# Fetch-Robotics-simulation
Repository for Fetch Robotics simulation and POCs

## Setup

1. Refer the documentation https://docs.fetchrobotics.com/gazebo.html


## Running the project
``
$ cd ros_ws
$ catkin init
$ catkin build
``

## Running samples
1. Simulate simple Robot
    1. Fetch : `roslaunch fetch_gazebo simulation.launch`
    2. Freight : `roslaunch fetch_gazebo simulation.launch robot:=freight`