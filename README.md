# Fetch-Robotics-simulation
Repository for Fetch Robotics simulation and POCs

## Setup

1. Refer the documentation https://docs.fetchrobotics.com/gazebo.html


## Running the project
```
$ cd ros_ws
$ catkin init
$ catkin build
```

## Running samples
1. Simulate simple Robot
    1. Fetch : `roslaunch sim_fetch_gazebo simulation.launch`
    2. Freight : `roslaunch sim_fetch_gazebo simulation.launch robot:=freight`
2. Demo Fetch Robot
    1. Run environment: `roslaunch sim_fetch_gazebo playground.launch`
    2. Load demo launch file: `roslaunch fetch_gazebo_demo demo.launch`
    3. Run Python file: `python2 demo.py`
3. Control with keyboard
    1. Run environment: `roslaunch sim_fetch_gazebo simulation.launch`
    2. Run Teleop keyboard: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
4. Manipulation of Robot
    1. Preconfigured with Rviz `roslaunch fetch_moveit_config demo.launch`
    2. Control with python
        1. Start simulation: `roslaunch sim_fetch_gazebo simulation.launch`
        2. Run python code: `python robot_motion.py`

    
