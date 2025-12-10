#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <mutex>

using Eigen::Matrix2d;
using Eigen::Vector2d;

struct TwoStateKF {
  Vector2d x;   // [theta, bias]
  Matrix2d P;
  Matrix2d Q;
  double R_motor;
  double R_odom;

  TwoStateKF();
  void init(double q_angle_rad2, double q_bias_rad2, double r_motor_rad2, double r_odom_rad2);
  void predict();
  void updateMotor(double z); // motor measurement: H = [1, -1]
  void updateOdom(double z);  // odom measurement:  H = [1, 0]
  static double wrap(double a);
};
