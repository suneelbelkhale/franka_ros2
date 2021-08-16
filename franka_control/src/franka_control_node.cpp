// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_manager/controller_manager.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>
#include <franka_msgs/action/ErrorRecovery.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/Trigger.hpp>

using franka_hw::ServiceContainer;
using namespace std::chrono_literals;


// action server
rclcpp_action::GoalResponse error_recovery_handle_goal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const franka_msgs::action::ErrorRecovery::Goal> goal) {
    (void)uuid;
    if (rclcpp::ok() && connected()) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse error_recovery_handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle,
  std::atomic_bool& has_error) {
  RCLCPP_INFO(robot_hw_nh_->get_logger(), "Cancel is not implemented! Will accept though...")
  return rclcpp_action::CancelResponse::ACCEPT;
}

void error_recovery_handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle,
  std::atomic_bool& has_error
  franka_hw::FrankaHW& franka_control) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{&execute, goal_handle, has_error, franka_control}.detach();
}

void execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle, 
  std::atomic_bool& has_error,
  franka_hw::FrankaHW& franka_control) {

  auto result = std::make_shared<franka_msgs::action::ErrorRecovery::Result>()

  if (rclcpp::ok() && franka_control.connected()) {
    try {
      // acquire robot lock
      std::lock_guard<std::mutex> lock(robot_mutex_);
      // blocking step
      robot_->automaticErrorRecovery();
      // error recovered => reset controller
      has_error = false;
      goal_handle->succeed(result);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Recovered from error");
    } catch (const franka::Exception& ex) {
      goal_handle->abort(result);
    }
  } else {
    goal_handle->abort(result);
  }
}


/* MAIN */

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> public_node_handle;
  auto node_handle = std::make_shared<rclcpp::Node>("franka_control_node");

  franka_hw::FrankaHW franka_control;
  if (!franka_control.init(public_node_handle, node_handle)) {
    RCLCPP_ERROR(node_handle->get_logger(), "franka_control_node: Failed to initialize FrankaHW class. Shutting down!");
    return 1;
  }

  auto services = std::make_unique<ServiceContainer>();
  rclcpp_action::Server<franka_msgs::action::ErrorRecovery>::SharedPtr
      recovery_action_server;

  std::atomic_bool has_error(false);

  auto connect = [&]() {
    franka_control.connect();
    std::lock_guard<std::mutex> lock(franka_control.robotMutex());
    auto& robot = franka_control.robot();

    services = std::make_unique<ServiceContainer>();
    franka_hw::setupServices(robot, franka_control.robotMutex(), node_handle, *services);
    recovery_action_server = node_handle->create_server<franka_msgs::action::ErrorRecovery>(node_handle, "error_recovery", 
      std::bind(&error_recovery_handle_goal, _1, _2),
      std::bind(&error_recovery_handle_cancel, _1, has_error),
      std::bind(&error_recovery_handle_accepted, _1, has_error, franka_control)
    );

    // Initialize robot state before loading any controller
    franka_control.update(robot.readOnce());
  };

  auto disconnect_handler = [&](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> bool {
    if (franka_control.controllerActive()) {
      response->success = 0u;
      response->message = "Controller is active. Cannot disconnect while a controller is running.";
      return true;
    }
    services.reset();
    recovery_action_server.reset();
    auto result = franka_control.disconnect();
    response->success = result ? 1u : 0u;
    response->message = result ? "" : "Failed to disconnect robot.";
    return true;
  };

  auto connect_handler = [&](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> bool {
    if (franka_control.connected()) {
      response->success = 0u;
      response->message = "Already connected to robot. Cannot connect twice.";
      return true;
    }

    connect();

    response->success = 1u;
    response->message = "";
    return true;
  };

  connect();

  rclcpp::Server<std_srvs::srv::Trigger>::SharedPtr connect_server =
      node_handle->create_service<std_srvs::srv::Trigger>(
          "connect", connect_handler);
  rclcpp::Server<std_srvs::srv::Trigger>::SharedPtr disconnect_server =
      node_handle->create_service<std_srvs::srv::Trigger>(
          "disconnect", disconnect_handler);

  controller_manager::ControllerManager control_manager(&franka_control, public_node_handle);

  while (rclcpp::ok()) {
    rclcpp::Time last_time = rclcpp::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!franka_control.controllerActive() || has_error) {
      if (franka_control.connected()) {
        try {
          std::lock_guard<std::mutex> lock(franka_control.robotMutex());
          franka_control.update(franka_control.robot().readOnce());
          rclcpp::Time now = rclcpp::Time::now();
          control_manager.update(now, now - last_time);
          franka_control.checkJointLimits();
          last_time = now;
        } catch (const std::logic_error& e) {
        }
      } else {
        std::this_thread::sleep_for(1ms);
      }

      if (!rclcpp::ok()) {
        return 0;
      }
    }

    if (franka_control.connected()) {
      try {
        // Run control loop. Will exit if the controller is switched.
        franka_control.control([&](const rclcpp::Time& now, const rclcpp::Duration& period) {
          if (period.toSec() == 0.0) {
            // Reset controllers before starting a motion
            control_manager.update(now, period, true);
            franka_control.checkJointLimits();
            franka_control.reset();
          } else {
            control_manager.update(now, period);
            franka_control.checkJointLimits();
            franka_control.enforceLimits(period);
          }
          return rclcpp::ok();
        });
      } catch (const franka::ControlException& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", e.what());
        has_error = true;
      }
    }
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("rclcpp"), 1, "franka_control, main loop");
  }

  return 0;
}
