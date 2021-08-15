find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(franka_hw_test
  launch/franka_hw_test.test
  main.cpp
  franka_hw_controller_switching_test.cpp
  franka_hw_interfaces_test.cpp
  franka_combinable_hw_controller_switching_test.cpp
)

ament_target_dependencies(franka_hw_test 
  actionlib
  controller_interface
  combined_robot_hw
  hardware_interface
  joint_limits_interface
  rclcpp
  std_srvs
  pluginlib
  urdf
  franka_msgs
  Franka
  )

# add_dependencies(franka_hw_test
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(franka_hw_test
#   ${catkin_LIBRARIES}
#   Franka::Franka
#   franka_hw
# )

# target_include_directories(franka_hw_test PUBLIC
#   ${catkin_INCLUDE_DIRS}
# )
