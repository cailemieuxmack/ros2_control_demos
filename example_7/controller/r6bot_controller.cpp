// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_example_7/r6bot_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>

// DEBUG
#include "controller.h"
#include <iostream>
#include <stdatomic.h>
#include <stdbool.h>
// #include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdio.h>
#include <errno.h>
// #include <semaphore.h>
// #include "controller.h"
#include <stdatomic.h>
#include <stdbool.h>

using config_type = controller_interface::interface_configuration_type;

namespace ros2_control_demo_example_7
{

// typedef struct {
//   int idx;
//   double values[1]; // u, (dx, da) <- not yet
// } Vote;

// typedef struct {
//   int idx;
//   double values[5]; // x,a,t temp(dx, da)
// } State_vote;


RobotController::RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RobotController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);

  // DEBUG
  setup_mapped_mem();

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
  auto callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
  {
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    //DEBUG
      // Extract and print the timestamp from the header
    std::cout << "Received trajectory with header timestamp: " 
        << "sec: " << traj_msg->header.stamp.sec 
        << ", nanosec: " << traj_msg->header.stamp.nanosec 
        << std::endl;
    new_msg_ = true;
  };

  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return CallbackReturn::SUCCESS;
}

void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
{
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_2.positions[i];
  }
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.velocities[i] =
      delta * point_2.velocities[i] + (1.0 - delta) * point_2.velocities[i];
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp)
{
  double traj_len = traj_msg.points.size();
  auto last_time = traj_msg.points[traj_len - 1].time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;

  size_t ind = cur_time.seconds() * (traj_len / total_time);
  ind = std::min(static_cast<double>(ind), traj_len - 2);
  double delta = cur_time.seconds() - ind * (total_time / traj_len);
  interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (new_msg_)
  {
    std::cout << "Update has recieved a new message" << std::endl;
    trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
    start_time_ = time;
    new_msg_ = false;
  }

  //std::cout << "Before traj_msg check " << std::endl;
  if (trajectory_msg_ != nullptr)
  {
    std::cout << "traj_msg exists " << std::endl;

    // DEBUG
    // Pass in the index and state
    tmp_state->idx = myIdx; //.store(5);//myIdx;
    //tmp_state->values[0] = tmp_vote->values[0]; // FIXME -> need to actually pass in the real state *******
    serialize_joint_trajectory(trajectory_msg_, tmp_state->value);

    std::cout << "Serialized" << std::endl;

    // Actually store the state in the maped memory
    for(int i = 0; i < 5; i++){
      state_vote->value = tmp_state->value;
    }
    // Store the index last
    state_vote->idx = tmp_state->idx;

    // Sleep so that the controller can run
    rclcpp::sleep_for(std::chrono::nanoseconds(100));

    // Get the proposed values
    tmp_vote->idx = data0->idx; 
    tmp_vote->values[0] = data0->values[0];
    //printf("idx: %d   value: %f\n", tmp_vote->idx, tmp_vote->values[0]);
    //printf("read: %d,   %f\n", tmp_vote->idx, tmp_vote->values[0]);

    std::cout << "idx recieved: " << tmp_vote->idx << std::endl;

    //if (tmp_vote->idx.load() >= myIdx) {
    if (tmp_vote->idx >= myIdx) {
      // We have a new message
      std::cout << "got: " << tmp_vote->values[0] << std::endl;
      // std::cout << sizeof(trajectory_msg_) << std::endl;

      // update index index
      myIdx = tmp_vote->idx; //.load();
      myIdx++;
    } else {
      // did not vote in time...
    }

    // std::cout << "end" << std::endl;
    // END DEBUG


    interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_);
    for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
    {
      joint_position_command_interface_[i].get().set_value(point_interp_.positions[i]);
    }
    for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
    {
      joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i]);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

// DEBUG
void RobotController::setup_mapped_mem() {

  // open or create the file with the proper permissions
  fd0 = open("_state", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
  // init the size of the file
  lseek (fd0, sizeof(Vote), SEEK_SET); 
  write (fd0, "", 1); 
  lseek (fd0, 0, SEEK_SET);

  // map the file into memory
  state_vote = static_cast<State_vote*>(mmap(NULL, sizeof(State_vote), PROT_WRITE, MAP_SHARED, fd0, 0));
  close(fd0);
  if (state_vote == MAP_FAILED){
      printf("error: %s\n", strerror(errno));
      exit(1);
  }
  printf("%p\n", state_vote);

  // // open or create the file with the proper permissions
  // fd1 = open("_actuation", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
  // // init the size of the file
  // lseek (fd1, sizeof(Vote), SEEK_SET); 
  // write (fd1, "", 1); 
  // lseek (fd1, 0, SEEK_SET);

  // // map the file into memory
  // actuation = mmap(NULL, sizeof(Vote), PROT_WRITE, MAP_SHARED, fd1, 0);
  // close(fd1);
  // if (actuation == -1){
  //     printf("error: %s\n", strerror(errno));
  //     exit(1);
  // }
  // printf("%p\n", actuation);

  fd1 = open("_data0", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
  // init the size of the file
  lseek (fd1, sizeof(Vote), SEEK_SET); 
  write (fd1, "", 1); 
  lseek (fd1, 0, SEEK_SET);

  // map the file into memory
  data0 = static_cast<Vote*>(mmap(NULL, sizeof(Vote), PROT_WRITE, MAP_SHARED, fd1, 0));
  close(fd1);
  if (data0 == MAP_FAILED){
      printf("error: %s\n", strerror(errno));
      exit(1);
  }

  //printf("vote: %d\n", sizeof(Vote));

  // // init the buffers
  // atomic_init(&actuation->idx, 0);
  // atomic_init(&actuation->values[0], 0.0);
  // for (int i = 0; i < 5; i++) {
  //     // std::atomic_init(&state_vote->values[i], 1.0);
  //     state_vote->values[i] = 1.0; //.store(1.0);
  // }
  // std::atomic_init(&state_vote->idx, 1);
  state_vote->idx = 1; //.store(1);

  myIdx = 0;

  // Open the results file after clearing it
  // remove("results.csv");
  // fd2 = open("results.csv", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);

  tmp_vote = static_cast<Vote*>(malloc(sizeof(Vote)));
  tmp_state = static_cast<State_vote*>(malloc(sizeof(State_vote)));

  // Do I have to init the tmp_state too?
  tmp_vote->idx = 0; //.store(0);
  tmp_vote->values[0] = 1.0; //.store(1.0);

  tmp_state->idx = 0; //.store(0);
  tmp_vote->values[0] = 1.0; //.store(1.0);

  have_actuation = false;

}

void RobotController::serialize_joint_trajectory(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& src, MappedJointTrajectory& dest) {
  if (!src) {
    std::cerr << "Error in serialize_joint_trajectory: trajectory_msg_ is null" << std::endl;
    return;
  }

  std::cout << "serialize_joint_trajectory: Serializing..." << std::endl;

  // Serialize joint names
  dest.joint_names_length = src->joint_names.size();
  for (size_t i = 0; i < src->joint_names.size(); ++i) {
      strncpy(dest.joint_names[i], src->joint_names[i].c_str(), sizeof(dest.joint_names[i]));
  }

  std::cout << "serialize_joint_trajectory: done with joint names" << std::endl;

  // Serialize points
  dest.points_length = src->points.size();
  for (size_t i = 0; i < src->points.size(); ++i) {
    // std::cout << "serialize_joint_trajectory: Inside main for loop" << std::endl;
      const auto& point = src->points[i];
      auto& mapped_point = dest.points[i];

      mapped_point.positions_length = point.positions.size();
      for (size_t j = 0; j < point.positions.size(); ++j) {
          std::cout << "serialize_joint_trajectory - position: " << j << " - " << point.positions[j] << std::endl;
          mapped_point.positions[j] = point.positions[j];
      }

      mapped_point.velocities_length = point.velocities.size();
      for (size_t j = 0; j < point.velocities.size(); ++j) {
          std::cout << "serialize_joint_trajectory - velo: " << point.velocities[j] << std::endl;
          mapped_point.velocities[j] = point.velocities[j];
      }

      mapped_point.accelerations_length = point.accelerations.size();
      for (size_t j = 0; j < point.accelerations.size(); ++j) {
          std::cout << "serialize_joint_trajectory - accel: " << point.accelerations[j] << std::endl;
          mapped_point.accelerations[j] = point.accelerations[j];
      }

      mapped_point.effort_length = point.effort.size();
      for (size_t j = 0; j < point.effort.size(); ++j) {
          std::cout << "serialize_joint_trajectory - effort: " << point.effort[j] << std::endl;
          mapped_point.effort[j] = point.effort[j];
      }

      // mapped_point.time_from_start_sec = point.time_from_start.sec;
      // mapped_point.time_from_start_nsec = point.time_from_start.nsec;
  }
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_7::RobotController, controller_interface::ControllerInterface)
