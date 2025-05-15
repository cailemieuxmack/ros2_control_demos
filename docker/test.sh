#!/bin/bash
test_id=$1
here_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

test_dir="$here_dir/test"
executable="/experiment/source/controller"

case $test_id in
    p1|p2|p3|p4|p5|p6|p7|p8|p9|p10|n1|n2)

        cp $test_dir/$test_id ./_state

        $executable

        oracle=$test_dir/output.$test_id
        python3 check_distance.py $oracle
        exit_code=$?
        exit $exit_code
        ;;

    *)
        exit 1
        ;;
esac