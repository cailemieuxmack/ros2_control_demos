#!/bin/bash

bad_num=$1

# max_idx for test case generation
max_idx=$2

here_dir=$(pwd)

cp c$bad_num/controller.c docker/controller_vulnerable.c

python3 make_testcases.py $1 $max_idx

cp -r docker/ repair_$bad_num/
cp repair.yml repair_$bad_num/

make

# FIXME: Update your path to Darjeeling
cd /home/jazbo/Vanderbilt/Research/git/Darjeeling

pipenv run /bin/bash /home/jazbo/Vanderbilt/Research/N-Version_APR/inverted_pendulum/repair_helper.sh $bad_num $here_dir