#! /bin/bash
dir=$1

gcc -c -o $dir/c1/controller.o $dir/c1/controller.c
g++ -c -o $dir/c1/controller_main.o $dir/c1/controller_main.cpp
g++ -o $dir/tmp1 $dir/c1/controller.o $dir/c1/controller_main.o