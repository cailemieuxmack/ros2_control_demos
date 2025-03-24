gcc -c -o controller.o controller.c
g++ -c -o controller_main.o controller_main.cpp
g++ -o tmp0 controller_main.o controller.o