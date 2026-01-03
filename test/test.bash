#!/bin/bash

cd ~/ros2_ws
colcon build --packages-select robosys2025_assignment_2

if [ $? == 0 ];then
    source ~/.bashrc
    ros2 launch robosys2025_assignment_2 hand_listen.launch.py
fi
