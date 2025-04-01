#! /bin/bash
dir=$1
gcc -c -o $dir/controller.o $dir/controller.c
g++ -c -o $dir/controller_main.o $dir/controller_main.cpp
g++ -o $dir/tmp0 $dir/controller.o $dir/controller_main.o