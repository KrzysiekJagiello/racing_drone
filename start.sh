#!/bin/bash
source ~/.bashrc

# Czyszczenie pozostalosci po poprzednich symulacjach
pkill -9 -f arducopter ; pkill -9 -f mavproxy ; pkill -9 -f gz ; pkill -9 ruby

# Uruchomienie managera procesow ROS 2
ros2 launch sitl_launcher sitl.launch.py