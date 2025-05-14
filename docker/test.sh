#!/bin/bash
test_id=$1
here_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
test_dir="$here_di
here_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
test_dir="$here_dir/test"
executable="/experiment/source/controller"
epsilon=0.1

case $test_id in
  p1|p2|p3|p4|p5|p6|p7|p8|p9|p10|n1|n2)

    guess=$($executable $(cat $test_dir/$test_id))
    actual=$(cat $test_dir/output.$test_id)
    cmp=$(echo "$actual - $guess" | bc)
    echo "TEST**: $actual   $guess"
    echo $cmp
    if (( $(echo "$cmp < 0.0" |bc -l) )); then
        cmp="${cmp:1}"
    fi
    if (( $(echo "$cmp < $epsilon" |bc -l) )); then
        exit 0
    fi
esac
exit 1