# ROS2 Controller Repair

## Description
```
This dirrecory contains an example implementation of N-version Automated Program Repair.
It is implemented on a robot arm control system written using ROS2. This robotic arm simulation comes from https://github.com/ros-controls/ros2_control_demos/tree/master/example_7.
The vulns/ dirrectory contains example vulnerable controller code and input generators to exploit them. 
So far, we have successfully repaired: vuln_segfault, vuln_fmtstr_crash, vuln_dos, and vuln_infinite_loop
```

## Dependencies

```
In order to run a repair, you must install Darjeeling from https://github.com/squaresLab/Darjeeling/
This project is designed to use commit 811337bc15186f0eaf49a4d15fb09441add2ff38.
Follow the dirrections given in the repository to install the dependencies and set up the pip environment. Then, update the path to Darjeeling
in the repair.sh file in this directory.
```

## Setup

Must be on Ubuntu 22.04 or higher for this to work. Has only been tested on Ubuntu 24.04.
```
./install_ros2_jazzy.sh       # or install ros2 jazzy manually using the dirrections at https://docs.ros.org/en/jazzy/Installation.html
./copy_vuln.sh safe
./build.sh
```

## Running an example

```
[copy a vulnerability into controller 0 and build everything]\
rm -rf log/ build/ install/
./copy_vuln.sh [name of vuln]
./build.sh
./build_controllers.sh

[start a workspace]
source ./source_workspace.sh

[in one workspace]
./start_controllers.sh
[in a second workspace]
./controller.sh
[in a third workspace]
python3 voter.py
[in a fourth workspace]
./send_trajectory.sh

[once a vulnerability is detected]
./repair.sh <controller num> <idx>
```
Here, the `controller num` means which controller to repair (./copy_vuln.sh makes controller 0 vulnerable) and `idx` refers to the highest index reached by the controllers. This will be reported in the output of both voter.py and ./start_controller.sh and can be clearly seen by running `ls results` and choosing the largest index present.

Repairs can also be triggered automatically by commenting out line 134 in voter.py.
```

[afterwards]
./cleanup.sh
```

## Vulnerabilities
| Vulnerability | Description |
|---------------|-------------|
| `bug_oob_array_access` | Off-by-one array access (`<=` instead of `<`) |
| `bug_type_casting` | Incorrect type casting in delta calculation |
| `bug_type_casting_2` | Unnecessary type casting in interpolation |
| `bug_floating_point` | Floating Point Exception from accidental divizion by 0 |
| `bug_interpolate_half_always` | Mathematical mistake. Always uses midpoint instead of scaling by delta |
| `vuln_stack_bof` | Stack buffer overflow in acceleration processing |
| `vuln_segfault` | Null pointer dereference in feature allowing inspecting memory |
| `vuln_infinite_loop` | Loop that increments but doesn't check negative values |
| `vuln_heap_bof` | Heap buffer overflow progressively corrupting more chunk metadata |
| `vuln_uaf` | Use-after-free between acceleration/effort processing |
| `vuln_fmtstr_crash` | Format string vulnerability reading invalid memory |
| `vuln_fmtstr_leak` | Format string vulnerability leaking stack data |
| `vuln_uaf` | Use after free error |

### Reference Implementations
| Implementation | Description |
|----------------|-------------|
| `safe` | Correct working implementation |
| `original_bug` | Original bug used for testing system |
