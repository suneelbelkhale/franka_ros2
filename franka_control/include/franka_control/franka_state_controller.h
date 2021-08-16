// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_msgs/msg/FrankaState.hpp>
#include <geometry_msgs/msg/WrenchStamped.hpp>
// #include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/msg/JointState.hpp>
#include <tf2_msgs/msg/TFMessage.hpp>

namespace franka_control {

/**
 * Controller to publish the robot state as ROS topics.
 */
class FrankaStateController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaStateInterface> {
 public:
  /**
   * Creates an instance of a FrankaStateController.
   */
  FrankaStateController() = default;

  /**
   * Initializes the controller with interfaces and publishers.
   *
   * @param[in] robot_hardware RobotHW instance to get a franka_hw::FrankaStateInterface from.
   * @param[in] root_node_handle Node handle in the controller_manager namespace.
   * @param[in] controller_node_handle Node handle in the controller namespace.
   */
  bool init(hardware_interface::RobotHW* robot_hardware,
            std::shared_ptr<rclcpp::Node> root_node_handle,
            std::shared_ptr<rclcpp::Node> controller_node_handle) override;

  /**
   * Reads the current robot state from the franka_hw::FrankaStateInterface and publishes it.
   *
   * @param[in] time Current ROS time.
   * @param[in] period Time since the last update.
   */
  void update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void publishFrankaStates(const rclcpp::Time& time);
  void publishJointStates(const rclcpp::Time& time);
  void publishTransforms(const rclcpp::Time& time);
  void publishExternalWrench(const rclcpp::Time& time);

  std::string arm_id_;

  franka_hw::FrankaStateInterface* franka_state_interface_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_{};

  //TODO REALTIME HERE
  // realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> publisher_transforms_;
  // realtime_tools::RealtimePublisher<franka_msgs::FrankaState> publisher_franka_states_;
  // realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_;
  // realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_desired_;
  // realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> publisher_external_wrench_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_transforms_;
  rclcpp::Publisher<franka_msgs::msg::FrankaState>::SharedPtr publisher_franka_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states_desired_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_external_wrench_;
  // franka_hw::TriggerRate trigger_publish_;

  rclcpp::TimerBase::SharedPtr timer_;

  franka::RobotState robot_state_;
  uint64_t sequence_number_ = 0;
  std::vector<std::string> joint_names_;
};

}  // namespace franka_control
