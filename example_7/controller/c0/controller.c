#include "../controller.h"
#include <stdio.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))


// InStruct in;
// OutStruct out;

double in[5];
double out[1];



// void interpolate_point(
//     const MappedJointTrajectoryPoint point_1,
//     const MappedJointTrajectoryPoint point_2,
//     MappedJointTrajectoryPoint * point_interp, double delta)
//   {
//     for (size_t i = 0; i < point_1.positions_length; i++)
//     {
//       (*point_interp).positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_2.positions[i];
//     }
//     for (size_t i = 0; i < point_1.positions_length; i++)
//     {
//       (*point_interp).velocities[i] =
//         delta * point_2.velocities[i] + (1.0 - delta) * point_2.velocities[i];
//     }
//   }
  
//   void interpolate_trajectory_point(
//     const MappedJointTrajectory traj_msg, const double cur_time_seconds,
//     MappedJointTrajectoryPoint * point_interp)
//   {
//     double traj_len = (double) traj_msg.points_length;
//     auto last_time = traj_msg.points[traj_len - 1].time_from_start;
//     double total_time = last_time.sec + last_time.nanosec * 1E-9;
  
//     size_t ind = cur_time_seconds * (traj_len / total_time);
//     ind = MIN( (double) ind, traj_len - 2);
//     double delta = cur_time_seconds - ind * (total_time / traj_len);
//     interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
//   }


int init() {
    printf("initializing controller...\n");
}


int step() {
    double msg = in[0];
    printf("msg: %f\n", msg);
    out[0] = msg + 1;
    // const trajectory_msgs::msg::JointTrajectory & traj_msg = in.traj_msg;
    // const rclcpp::Duration & cur_time = in.time - in.start_time;

    // interpolate_trajectory_point(traj_msg, cur_time, out.point_interp);
}
