// Copyright 2019 ROBOTIS CO., LTD.
// Author: Darby Lim

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "odometry.hpp"

namespace mybot
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace mybot
