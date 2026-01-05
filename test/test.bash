#!/bin/bash
# SPDX-FileCopyrightText: 2025-2026 Shogo Yamashita
# SPDX-License-Identifier: Apache-2.0

cd ~/ros2_ws
colcon build --packages-select robosys2025_assignment_2

if [ $? == 0 ];then
    source ~/.bashrc
    #ros2 launch robosys2025_assignment_2 janken_out.launch.py
    ros2 launch robosys2025_assignment_2 number_out.launch.py
fi
