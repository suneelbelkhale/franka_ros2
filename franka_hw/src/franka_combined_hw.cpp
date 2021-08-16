// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_combined_hw.h>

#include <algorithm>
#include <memory>

#include <std_srvs/Trigger.h>

#include <franka_hw/franka_combinable_hw.h>
#include <franka_hw/franka_hw.h>
#include <franka_msgs/ErrorRecoveryAction.h>

namespace franka_hw {

FrankaCombinedHW::FrankaCombinedHW() = default;

bool FrankaCombinedHW::init(std::shared_ptr<rclcpp::Node> root_nh, std::shared_ptr<rclcpp::Node> robot_hw_nh) {
  bool success = CombinedRobotHW::init(root_nh, robot_hw_nh);
  robot_hw_nh_ = robot_hw_nh;
  // Error recovery server for all FrankaHWs
  combined_recovery_action_server_ = rclcpp_action::create_server<franka_msgs::action::ErrorRecovery>(robot_hw_nh, "error_recovery", 
      std::bind(&FrankaCombinedHW::error_recovery_handle_goal, this, _1, _2),
      std::bind(&FrankaCombinedHW::error_recovery_handle_cancel, this, _1),
      std::bind(&FrankaCombinedHW::error_recovery_handle_accepted, this, _1)
    );

  connect_server_ =
    robot_hw_nh_->create_service<std_srvs::srv::Trigger>(
        "connect",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
          try {
            connect();
            RCLCPP_INFO(robot_hw_nh_->get_logger(), "FrankaCombinedHW: successfully connected robots.");
            response.success = 1u;
            response.message = "";
          } catch (const std::exception& e) {
            RCLCPP_INFO(robot_hw_nh_->get_logger(), "Combined: exception %s", e.what());
            response.success = 0u;
            response.message =
                "FrankaCombinedHW: Failed to connect robot: " + std::string(e.what());
          }
        }
    );

  disconnect_server_ =
    robot_hw_nh_->create_service<std_srvs::srv::Trigger>(
        "disconnect",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
          bool success = disconnect();
          response->success = success ? 1u : 0u;
          response->message = success
                                 ? "FrankaCombinedHW: Successfully disconnected robots."
                                 : "FrankaCombinedHW: Failed to disconnect robots. All active "
                                   "controllers must be stopped before you can disconnect.";
          if (success) {
            RCLCPP_INFO(robot_hw_nh_->get_logger(), "%s", response->message.c_str());
          } else {
            RCLCPP_ERROR(robot_hw_nh_->get_logger(), "%s", response->message.c_str());
          }
        }
    );

  return success;
}

// action server

rclcpp_action::GoalResponse FrankaCombinedHW::error_recovery_handle_goal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const franka_msgs::action::ErrorRecovery::Goal> goal) {
    (void)uuid;
    if (rclcpp::ok() && !is_recovering_) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
}
  
rclcpp_action::CancelResponse FrankaCombinedHW::error_recovery_handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle) {
  RCLCPP_INFO(robot_hw_nh_->get_logger(), "Cancel is not implemented! Will accept though...")
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FrankaCombinedHW::error_recovery_handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&FrankaCombinedHW::execute, this, _1), goal_handle}.detach();
}

void FrankaCombinedHW::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle) {

  auto result = std::make_shared<franka_msgs::action::ErrorRecovery::Result>()

  try {
    is_recovering_ = true;
    for (const auto& robot_hw : robot_hw_list_) {
      auto* franka_combinable_hw_ptr =
          dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
      if (franka_combinable_hw_ptr != nullptr && franka_combinable_hw_ptr->connected()) {
        franka_combinable_hw_ptr->resetError();
      } else {
        RCLCPP_ERROR("FrankaCombinedHW: failed to reset error. Is the robot connected?");
        is_recovering_ = false;
        goal_handle->abort(result);
        return;
      }
    }
    is_recovering_ = false;
    goal_handle->succeed(result);
  } catch (const franka::Exception& ex) {
    is_recovering_ = false;
    goal_handle->abort(result);
  }
}

void FrankaCombinedHW::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Call the read method of the single RobotHW objects.
  CombinedRobotHW::read(time, period);
  handleError();
}

bool FrankaCombinedHW::controllerNeedsReset() {
  // Check if any of the RobotHW object needs a controller reset
  bool controller_reset = false;
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      controller_reset = controller_reset || franka_combinable_hw_ptr->controllerNeedsReset();
    } else {
      ROS_ERROR("FrankaCombinedHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
      return false;
    }
  }
  return controller_reset;
}

void FrankaCombinedHW::handleError() {
  // Trigger error state of all other RobotHW objects when one of them has a error.
  if (hasError() && !is_recovering_) {
    triggerError();
  }
}

bool FrankaCombinedHW::hasError() {
  bool has_error = false;
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      has_error = has_error || franka_combinable_hw_ptr->hasError();
    } else {
      RCLCPP_ERROR(robot_hw_nh_->get_logger(), "FrankaCombinedHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
      return false;
    }
  }
  return has_error;
}

void FrankaCombinedHW::triggerError() {
  // Trigger error state of all RobotHW objects.
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      franka_combinable_hw_ptr->triggerError();
    } else {
      RCLCPP_ERROR(robot_hw_nh_->get_logger(), "FrankaCombinedHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
    }
  }
}

void FrankaCombinedHW::connect() {
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr && !franka_combinable_hw_ptr->connected()) {
      franka_combinable_hw_ptr->connect();
    }
  }
}

bool FrankaCombinedHW::disconnect() {
  // Ensure all robots are disconnectable (not running a controller)
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr && franka_combinable_hw_ptr->controllerActive()) {
      return false;
    }
  }

  // Only if all robots are in fact disconnectable, disconnecting them.
  // Fail and abort if any robot cannot be disconnected.
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr && !franka_combinable_hw_ptr->disconnect()) {
      return false;
    }
  }

  return true;
}

}  // namespace franka_hw
