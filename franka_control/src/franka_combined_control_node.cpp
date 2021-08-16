// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <controller_manager/controller_manager.h>
#include <franka_hw/franka_combined_hw.h>
#include <rclcpp/rclcpp.hpp>

#include <franka/control_tools.h>
#include <sched.h>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto private_node_handle = std::make_shared<rclcpp::Node>("franka_combined_control_node");

  franka_hw::FrankaCombinedHW franka_control;
  if (!franka_control.init(private_node_handle, private_node_handle)) {
    ROS_ERROR("franka_combined_control_node:: Initialization of FrankaCombinedHW failed!");
    return 1;
  }

  // set current thread to real-time priority
  std::string error_message;
  if (!franka::setCurrentThreadToHighestSchedulerPriority(&error_message)) {
    ROS_ERROR("franka_combined_control_node: Failed to set thread priority to real-time. Error: %s",
              error_message.c_str());
    return 1;
  }

  controller_manager::ControllerManager cm(&franka_control, private_node_handle);
  rclcpp::Duration period(0.001);
  rclcpp::Rate rate(period);

  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::Time now = rclcpp::Time::now();
    franka_control.read(now, period);
    cm.update(now, period, franka_control.controllerNeedsReset());
    franka_control.write(now, period);
  }

  return 0;
}
