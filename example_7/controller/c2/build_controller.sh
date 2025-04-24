#! /bin/bash
dir=$1

gcc -c -o $dir/c2/controller.o $dir/c2/controller.c
g++ -c -o $dir/c2/controller_main.o $dir/c2/controller_main.cpp
g++ -o $dir/tmp2 $dir/c2/controller.o $dir/c2/controller_main.o