// ROS2 Stuff

// #include "ros2_control_demo_example_7/r6bot_controller.hpp"

// #include <stddef.h>
// #include <algorithm>
// #include <memory>
// #include <string>
// #include <vector>

// #include "rclcpp/qos.hpp"
// #include "rclcpp/time.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"

//extern double in[];
//extern double out[];

// typedef struct {
//     // rclcpp::Time time;
//     // rclcpp::Time start_time;
//     // trajectory_msgs::msg::JointTrajectory trajectory_msg;
//     int msg;
// } InStruct;

// typedef struct {
//     // trajectory_msgs::msg::JointTrajectoryPoint point_interp;
//     int rsp;
// } OutStruct;

extern double in[];
extern double out[];


int init();
int step();
