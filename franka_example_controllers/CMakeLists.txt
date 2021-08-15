cmake_minimum_required(VERSION 3.5)
project(franka_example_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(franka_hw REQUIRED)
find_package(franka_gripper REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(tf REQUIRED)
find_package(tf_conversions REQUIRED)
find_package(message_generation REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  JointTorqueComparison.msg
)

# TODO
# generate_dynamic_reconfigure_options(
#   cfg/compliance_param.cfg
#   cfg/desired_mass_param.cfg
#   cfg/dual_arm_compliance_param.cfg
#   cfg/teleop_param.cfg
#   cfg/teleop_gripper_param.cfg
# )

ament_export_dependencies(
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_runtime
  pluginlib
  realtime_tools
  rclcpp
  Franka
)

ament_export_include_directories(include)

ament_export_libraries(franka_example_controllers)


add_library(franka_example_controllers
  src/elbow_example_controller.cpp
  src/cartesian_pose_example_controller.cpp
  src/cartesian_velocity_example_controller.cpp
  src/joint_position_example_controller.cpp
  src/joint_velocity_example_controller.cpp
  src/model_example_controller.cpp
  src/joint_impedance_example_controller.cpp
  src/cartesian_impedance_example_controller.cpp
  src/force_example_controller.cpp
  src/dual_arm_cartesian_impedance_example_controller.cpp
  src/teleop_joint_pd_example_controller.cpp
)

# add_dependencies(franka_example_controllers
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_generate_messages_cpp
#   ${PROJECT_NAME}_gencpp
#   ${PROJECT_NAME}_gencfg
# )

# target_link_libraries(franka_example_controllers PUBLIC
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_example_controllers SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )
# target_include_directories(franka_example_controllers PUBLIC
#   include
# )

ament_target_dependencies(franka_example_controllers
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_runtime
  pluginlib
  realtime_tools
  rclcpp
  Franka
)

add_executable(teleop_gripper_node
  src/teleop_gripper_node.cpp
)
target_include_directories(teleop_gripper_node PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(teleop_gripper_node PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

ament_target_dependencies(teleop_gripper_node
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_runtime
  pluginlib
  realtime_tools
  rclcpp
  Franka
)

## Installation
install(TARGETS franka_example_controllers
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES franka_example_controllers_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# TODO add an entry point for python scripts?
# catkin_install_python(
#   PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_example_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_example_controllers
    FILES ${SOURCES}
    DEPENDS franka_example_controllers
  )
endif()

ament_package()