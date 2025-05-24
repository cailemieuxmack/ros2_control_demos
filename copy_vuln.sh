#! /bin/bash

vuln=$1

cp vulns/$vuln/controller.c example_7/controller/c0/controller.c

cp vulns/$vuln/send_trajectory.cpp example_7/reference_generator/send_trajectory.cpp