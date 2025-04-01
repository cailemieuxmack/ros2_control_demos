#! /bin/bash
dir=$1

gcc -c -o $dir/c0/controller.o $dir/c0/controller.c
g++ -c -o $dir/c0/controller_main.o $dir/c0/controller_main.cpp
g++ -o $dir/tmp0 $dir/c0/controller.o $dir/c0/controller_main.o