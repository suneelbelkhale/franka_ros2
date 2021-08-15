cmake_minimum_required(VERSION 3.5)
project(franka_msgs)

find_package(catkin REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  Errors.msg 
  FrankaState.msg
  SetCartesianImpedance.srv
  SetEEFrame.srv
  SetForceTorqueCollisionBehavior.srv
  SetFullCollisionBehavior.srv
  SetJointImpedance.srv
  SetKFrame.srv
  SetLoad.srv
  ErrorRecovery.action
  DEPENDENCIES std_msgs actionlib_msgs
)

ament_export_dependencies(std_msgs actionlib_msgs)
ament_package()
