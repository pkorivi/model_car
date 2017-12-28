#!/bin/bash
reset
catkin_make MotionPlanner
source devel/setup.bash
roslaunch motion_planner mp_test.launch
