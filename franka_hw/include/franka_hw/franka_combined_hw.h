// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <combined_robot_hw/combined_robot_hw.h>
#include <franka_hw/franka_combinable_hw.h>
#include <franka_msgs/ErrorRecoveryAction.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/time.h>

#include <memory>

namespace franka_hw {

class FrankaCombinedHW : public combined_robot_hw::CombinedRobotHW {
 public:
  /**
   * Creates an instance of CombinedFrankaHW.
   */
  FrankaCombinedHW();

  ~FrankaCombinedHW() override = default;  // NOLINT (clang-analyzer-optin.cplusplus.VirtualCall)

  /**
   * The init function is called to initialize the CombinedFrankaHW from a
   * non-realtime thread.
   *
   * @param[in] root_nh A NodeHandle in the root of the caller namespace.
   * @param[in] robot_hw_nh A NodeHandle in the namespace from which the RobotHW.
   * should read its configuration.
   * @return True if initialization was successful.
   */
  bool init(rstd::shared_ptr<rclcpp::Node> root_nh, std::shared_ptr<rclcpp::Node> robot_hw_nh) override;

  /**
   * Reads data from the robot HW
   *
   * @param[in] time The current time.
   * @param[in] period The time passed since the last call to \ref read.
   */
  void read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * Checks whether the controller needs to be reset.
   *
   * @return True if the controllers needs to be reset, false otherwise.
   */
  bool controllerNeedsReset();

  /**
   * Calls connect on all hardware classes that are of type `FrankaCombinableHW`.
   */
  void connect();

  /**
   * Tries to disconnect on all hardware classes that are of type `FrankaCombinableHW`.
   * @return true if successful, false otherwise.
   */
  bool disconnect();

  // action server
  rclcpp_action::GoalResponse error_recovery_handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const franka_msgs::action::ErrorRecovery::Goal> goal);
  
  rclcpp_action::CancelResponse error_recovery_handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle);

  void error_recovery_handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle);
  
  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle);


 protected:
  rclcpp_action::Server<franka_msgs::action::ErrorRecovery>::SharedPtr combined_recovery_action_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_server_;
  
  std::shared_ptr<rclcpp::Node> robot_hw_nh;

 private:
  void handleError();
  bool hasError();
  void triggerError();
  bool is_recovering_{false};
};

}  // namespace franka_hw
