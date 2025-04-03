#! /bin/bash

dir="/home/ros2_ws/src/ros2_control_demos/example_7/controller"

# $dir/c0/build_controller.sh $dir

gcc -c -o $dir/c0/controller.o $dir/c0/controller.c
g++ -c -o $dir/c0/controller_main.o $dir/c0/controller_main.cpp
g++ -o $dir/tmp0 $dir/c0/controller.o $dir/c0/controller_main.o

cp $dir/tmp0 /home/ros2_ws/tmp0
