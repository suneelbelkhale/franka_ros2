// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/JointState.hpp>

#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>

namespace {

template <typename T>
void handleErrors(std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle,
                  std::function<bool(const std::shared_ptr<T::Goal>)> handler) {
  auto result = std::make_shared<T::Result>();
  try {
    result->success = handler(goal_handle->get_goal());
    server->succeed(result);
  } catch (const franka::Exception& ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "" << ex.what());
    result->success = false;
    result->error = ex.what();
    server->abort(result);
  }
}


}  // anonymous namespace

// functions
using franka_gripper::GraspEpsilon;
using franka_gripper::gripperCommandExecuteCallback;
using franka_gripper::grasp;
using franka_gripper::homing;
using franka_gripper::move;
using franka_gripper::stop;

// actions
using control_msgs::action::GripperCommand;
using franka_gripper::action::Grasp;
using franka_gripper::action::Homing;
using franka_gripper::action::Move;
using franka_gripper::action::Stop;

using franka_gripper::updateGripperState;


// publishing callback
void gripper_joint_state_publish_timer(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher, 
  std::mutex& gripper_state_mutex,
  franka::GripperState& gripper_state) {
  auto joint_states = std::make_shared<sensor_msgs::msg::JointState>()
  if (gripper_state_mutex.try_lock()) {
      joint_states->header.stamp = rclcpp::Time::now();
      joint_states->name.push_back(joint_names[0]);
      joint_states->name.push_back(joint_names[1]);
      joint_states->position.push_back(gripper_state.width * 0.5);
      joint_states->position.push_back(gripper_state.width * 0.5);
      joint_states->velocity.push_back(0.0);
      joint_states->velocity.push_back(0.0);
      joint_states->effort.push_back(0.0);
      joint_states->effort.push_back(0.0);
      publisher->publish(joint_states);
      gripper_state_mutex.unlock();
    }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("franka_gripper_node");
  node_handle->declare_parameter<std::string>("robot_ip", "");
  node_handle->declare_parameter<double>("default_speed", 0.1);
  node_handle->declare_parameter<std::map<std::string, double>>("default_grasp_epsilon", std::map<std::string, double>());
  node_handle->declare_parameter<double>("publish_rate", 30.);
  node_handle->declare_parameter<std::vector<std::string>>("joint_names", {});
  node_handle->declare_parameter<bool>("stop_at_shutdown", false);

  std::string robot_ip;
  if (!node_handle->get_parameter<std::string>("robot_ip", robot_ip)) {
    RCLCPP_ERROR(node_handle->get_logger(), "franka_gripper_node: Could not parse robot_ip parameter");
    return -1;
  }

  double default_speed(0.1);
  if (node_handle->get_parameter<double>("default_speed", default_speed)) {
    RCLCPP_INFO_STREAM(node_handle->get_logger(), "franka_gripper_node: Found default_speed " << default_speed);
  }

  GraspEpsilon default_grasp_epsilon;
  default_grasp_epsilon.inner = 0.005;
  default_grasp_epsilon.outer = 0.005;
  std::map<std::string, double> epsilon_map;
  if (node_handle->get_parameter<std::map<std::string, double>>("default_grasp_epsilon", epsilon_map)) {
    RCLCPP_INFO_STREAM(node_handle->get_logger(), "franka_gripper_node: Found default_grasp_epsilon "
                    << "inner: " << epsilon_map["inner"] << ", outer: " << epsilon_map["outer"]);
    default_grasp_epsilon.inner = epsilon_map["inner"];
    default_grasp_epsilon.outer = epsilon_map["outer"];
  }

  franka::Gripper gripper(robot_ip);

  auto homing_handler = [&gripper](auto&& goal) { return homing(gripper, goal); };
  auto stop_handler = [&gripper](auto&& goal) { return stop(gripper, goal); };
  auto grasp_handler = [&gripper](auto&& goal) { return grasp(gripper, goal); };
  auto move_handler = [&gripper](auto&& goal) { return move(gripper, goal); };

  auto homing_action_server = node_handle->create_server<Homing>(
      node_handle, "homing",
      [=, &homing_action_server](auto&& goal) {
        return handleErrors<Homing>(homing_handler, goal);
      },
      false);

  auto stop_action_server = node_handle->create_server<Stop>(
      node_handle, "stop",
      [=, &stop_action_server](auto&& goal) {
        return handleErrors<Stop>(stop_handler, goal);
      },
      false);

  auto move_action_server = node_handle->create_server<Move>(
      node_handle, "move",
      [=, &move_action_server](auto&& goal) {
        return handleErrors<Move>(move_handler, goal);
      },
      false);

  auto grasp_action_server = node_handle->create_server<Grasp>(
      node_handle, "grasp",
      [=, &grasp_action_server](auto&& goal) {
        return handleErrors<Grasp>(grasp_handler, goal);
      },
      false);

  auto gripper_command_action_server = node_handle->create_server<GripperCommand>(
      node_handle, "gripper_action",
      [=, &gripper, &gripper_command_action_server](auto&& goal) {
        return gripperCommandExecuteCallback(gripper, default_grasp_epsilon, default_speed, goal);
      },
      false);

  double publish_rate(30.0);
  if (!node_handle->get_parameter<double>("publish_rate", publish_rate)) {
    RCLCPP_INFO_STREAM(node_handle->get_logger(), "franka_gripper_node: Could not find parameter publish_rate. Defaulting to "
                    << publish_rate);
  }

  std::vector<std::string> joint_names;
  if (!node_handle->get_parameter<std::vector<std::string>>("joint_names", joint_names)) {
    RCLCPP_ERROR(node_handle->get_logger(), "franka_gripper_node: Could not parse joint_names!");
    return -1;
  }
  if (joint_names.size() != 2) {
    RCLCPP_ERROR(node_handle->get_logger(), "franka_gripper_node: Got wrong number of joint_names!");
    return -1;
  }

  bool stop_at_shutdown(false);
  if (!node_handle->get_parameter<bool>("stop_at_shutdown", stop_at_shutdown)) {
    RCLCPP_INFO_STREAM(node_handle->get_logger(), "franka_gripper_node: Could not find parameter stop_at_shutdown. Defaulting to "
                    << std::boolalpha << stop_at_shutdown);
  }

  franka::GripperState gripper_state;
  std::mutex gripper_state_mutex;
  // gripper read thread
  std::thread read_thread([&gripper_state, &gripper, &gripper_state_mutex]() {
    rclcpp::Rate read_rate(10);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> _(gripper_state_mutex);
        updateGripperState(gripper, &gripper_state);
      }
      read_rate.sleep();
    }
  });

  // publishing gripper joint states
  auto gripper_state_publisher =
      node_handle->ceate_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  // publish dt = 1 / publish rate
  auto timer = node_handle->create_wall_timer(
        1. / publish_rate, std::bind(gripper_joint_state_publish_timer, gripper_state_publisher, gripper_state_mutex, gripper_state));
  
  // spin blocking
  rclcpp::spin();

  read_thread.join();
  if (stop_at_shutdown) {
    gripper.stop();
  }
  return 0;
}
