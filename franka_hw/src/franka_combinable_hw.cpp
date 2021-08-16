// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_combinable_hw.h>

#include <chrono>
#include <thread>

#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <pluginlib/class_list_macros.h>

#include <franka_hw/services.h>

using namespace std::chrono_literals;

namespace franka_hw {

FrankaCombinableHW::FrankaCombinableHW() : has_error_(false), error_recovered_(false) {}

bool FrankaCombinableHW::init(std::shared_ptr<rclcpp::Node> root_nh, std::shared_ptr<rclcpp::Node> robot_hw_nh) {
  robot_hw_nh_ = robot_hw_nh;
  return FrankaHW::init(root_nh, robot_hw_nh);
}

void FrankaCombinableHW::initROSInterfaces(std::shared_ptr<rclcpp::Node> robot_hw_nh) {
  setupJointStateInterface(robot_state_ros_);
  setupJointCommandInterface(effort_joint_command_ros_.tau_J, robot_state_ros_, false,
                             effort_joint_interface_);
  setupLimitInterface<joint_limits_interface::EffortJointSoftLimitsHandle>(
      effort_joint_limit_interface_, effort_joint_interface_);
  setupFrankaStateInterface(robot_state_ros_);
  setupFrankaModelInterface(robot_state_ros_);

  has_error_pub_ = robot_hw_nh->create_publisher<std_msgs::msg::Bool>("has_error", 1);
  publishErrorState(has_error_);

  setupServicesAndActionServers(robot_hw_nh);
}

void FrankaCombinableHW::initRobot() {
  FrankaHW::initRobot();
  control_loop_thread_ = std::make_unique<std::thread>(&FrankaCombinableHW::controlLoop, this);
}

void FrankaCombinableHW::publishErrorState(const bool error) {
  std_msgs::msgs::Bool msg;
  msg.data = static_cast<int>(error);
  has_error_pub_->publish(msg);
}

void FrankaCombinableHW::controlLoop() {
  while (rclcpp::ok()) {
    auto last_time = rclcpp::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!controllerActive() || has_error_) {
      if (!controllerActive()) {
        RCLCPP_DEBUG_THROTTLE(robot_hw_nh_->get_logger(), 1, "FrankaCombinableHW::%s::control_loop(): controller is not active.",
                           arm_id_.c_str());
      }
      if (has_error_) {
        RCLCPP_DEBUG_THROTTLE(robot_hw_nh_->get_logger(), 1, "FrankaCombinableHW::%s::control_loop(): an error has occured.",
                           arm_id_.c_str());
      }

      if (initialized_) {
        checkJointLimits();
      }
      {
        std::lock_guard<std::mutex> robot_lock(robot_mutex_);
        if (connected()) {
          std::lock_guard<std::mutex> ros_state_lock(ros_state_mutex_);
          std::lock_guard<std::mutex> libfranka_state_lock(libfranka_state_mutex_);
          robot_state_libfranka_ = robot_->readOnce();
          robot_state_ros_ = robot_->readOnce();
        }
      }

      if (!rclcpp::ok()) {
        return;
      }
      std::this_thread::sleep_for(1ms);
    }
    RCLCPP_INFO(robot_hw_nh_->get_logger(), "FrankaCombinableHW::%s::control_loop(): controller is active.", arm_id_.c_str());

    // Reset commands
    {
      std::lock_guard<std::mutex> command_lock(libfranka_cmd_mutex_);
      effort_joint_command_libfranka_ = franka::Torques({0., 0., 0., 0., 0., 0., 0.});
    }

    try {
      if (connected()) {
        control();
      }
    } catch (const franka::ControlException& e) {
      // Reflex could be caught and it needs to wait for automatic error recovery
      RCLCPP_ERROR(robot_hw_nh_->get_logger(), "%s: %s", arm_id_.c_str(), e.what());
      has_error_ = true;
      publishErrorState(has_error_);
    }
  }
}

void FrankaCombinableHW::setupServicesAndActionServers(std::shared_ptr<rclcpp::Node> node_handle) {
  if (!connected()) {
    RCLCPP_ERROR(robot_hw_nh_->get_logger(), 
        "FrankaCombinableHW::setupServicesAndActionServers: Cannot create services without "
        "connected robot.");
    return;
  }

  services_ = std::make_unique<ServiceContainer>();
  setupServices(*robot_, robot_mutex_, node_handle, *services_);

  if (!recovery_action_server_) {
    recovery_action_server_ = node_handle->create_server<franka_msgs::action::ErrorRecovery>(node_handle, "error_recovery", 
      std::bind(&FrankaCombinableHW::error_recovery_handle_goal, this, _1, _2),
      std::bind(&FrankaCombinableHW::error_recovery_handle_cancel, this, _1),
      std::bind(&FrankaCombinableHW::error_recovery_handle_accepted, this, _1)
    );
  }
}

// action server

rclcpp_action::GoalResponse FrankaCombinableHW::error_recovery_handle_goal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const franka_msgs::action::ErrorRecovery::Goal> goal) {
    (void)uuid;
    if (rclcpp::ok() && connected()) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
}
  
rclcpp_action::CancelResponse FrankaCombinableHW::error_recovery_handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle) {
  RCLCPP_INFO(robot_hw_nh_->get_logger(), robot_hw_nh_->get_logger(), "Cancel is not implemented! Will accept though...")
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FrankaCombinableHW::error_recovery_handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&FrankaCombinableHW::execute, this, _1), goal_handle}.detach();
}

void FrankaCombinableHW::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>> goal_handle) {

  auto result = std::make_shared<franka_msgs::action::ErrorRecovery::Result>()

  if (rclcpp::ok() && connected()) {
    try {
      // acquire robot lock
      std::lock_guard<std::mutex> lock(robot_mutex_);
      // blocking step
      robot_->automaticErrorRecovery();
      // error recovered => reset controller
      if (has_error_) {
        error_recovered_ = true;
      }
      has_error_ = false;
      publishErrorState(has_error_);
      goal_handle->succeed(result);
    } catch (const franka::Exception& ex) {
      goal_handle->abort(result);
    }
  } else {
    goal_handle->abort(result);
  }
}

void FrankaCombinableHW::connect() {
  FrankaHW::connect();
  setupServicesAndActionServers(robot_hw_nh_);
}

bool FrankaCombinableHW::disconnect() {
  if (controllerActive()) {
    RCLCPP_ERROR(robot_hw_nh_->get_logger(), "FrankaHW: Rejected attempt to disconnect while controller is still running!");
    return false;
  }
  recovery_action_server_.reset();
  services_.reset();
  return FrankaHW::disconnect();
}

void FrankaCombinableHW::control(  // NOLINT (google-default-arguments)
    const std::function<bool(const rclcpp::Time&, const rclcpp::Duration&)>& /*ros_callback*/) {
  if (!controller_active_) {
    return;
  }
  auto empty_method = [](const franka::RobotState&, franka::Duration) { return true; };
  run_function_(*robot_, empty_method);
}

bool FrankaCombinableHW::checkForConflict(
    const std::list<hardware_interface::ControllerInfo>& info) const {
  ResourceWithClaimsMap resource_map = getResourceMap(info);

  if (hasConflictingMultiClaim(resource_map)) {
    return true;
  }

  ArmClaimedMap arm_claim_map;
  if (!getArmClaimedMap(resource_map, arm_claim_map)) {
    RCLCPP_ERROR(robot_hw_nh_->get_logger(), "FrankaCombinableHW: Unknown interface claimed. Conflict!");
    return true;
  }

  // check for any claim to trajectory interfaces (non-torque) which are not supported.
  if (hasTrajectoryClaim(arm_claim_map, arm_id_)) {
    RCLCPP_ERROR_STREAM(robot_hw_nh_->get_logger(), "FrankaCombinableHW: Invalid claim joint position or velocity interface."
                     << "Note: joint position and joint velocity interfaces are not supported"
                     << " in FrankaCombinableHW. Arm:" << arm_id_ << ". Conflict!");
    return true;
  }

  return partiallyClaimsArmJoints(arm_claim_map, arm_id_);
}

void FrankaCombinableHW::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
  controller_needs_reset_ = bool(error_recovered_);
  FrankaHW::read(time, period);
}

void FrankaCombinableHW::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // if flag `controller_needs_reset_` was updated, then controller_manager. update(...,
  // reset_controller) must
  // have been executed to reset the controller.
  if (controller_needs_reset_ && error_recovered_) {
    controller_needs_reset_ = false;
    error_recovered_ = false;
  }

  enforceLimits(period);

  FrankaHW::write(time, period);
}

std::string FrankaCombinableHW::getArmID() const noexcept {
  return arm_id_;
}

void FrankaCombinableHW::triggerError() {
  has_error_ = true;
  publishErrorState(has_error_);
}

bool FrankaCombinableHW::hasError() const noexcept {
  return has_error_;
}

void FrankaCombinableHW::resetError() {
  if (connected()) {
    robot_->automaticErrorRecovery();
  }
  // error recovered => reset controller
  if (has_error_) {
    error_recovered_ = true;
  }
  has_error_ = false;
  publishErrorState(has_error_);
}

bool FrankaCombinableHW::controllerNeedsReset() const noexcept {
  return controller_needs_reset_;
}

bool FrankaCombinableHW::setRunFunction(const ControlMode& requested_control_mode,
                                        const bool limit_rate,
                                        const double cutoff_frequency,
                                        const franka::ControllerMode /*internal_controller*/) {
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  if (requested_control_mode == ControlMode::None) {
    return true;
  }
  if (requested_control_mode == ControlMode::JointTorque) {
    run_function_ = [this, limit_rate, cutoff_frequency](franka::Robot& robot,
                                                         Callback /*callback*/) {
      std::lock_guard<std::mutex> lock(robot_mutex_);
      robot.control(std::bind(&FrankaCombinableHW::libfrankaUpdateCallback<franka::Torques>, this,
                              std::cref(effort_joint_command_libfranka_), std::placeholders::_1,
                              std::placeholders::_2),
                    limit_rate, cutoff_frequency);
    };
    return true;
  }

  RCLCPP_ERROR(robot_hw_nh_->get_logger(), "FrankaCombinableHW: No valid control mode selected; cannot set run function.");
  return false;
}

}  // namespace franka_hw

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaCombinableHW, hardware_interface::RobotHW)
