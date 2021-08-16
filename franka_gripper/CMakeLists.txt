cmake_minimum_required(VERSION 3.5)
project(franka_gripper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(message_generation REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(xmlrpcpp REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(Franka 0.7.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  GraspEpsilon.msg
  Grasp.action
  Homing.action
  Stop.action
  Move.action
  DEPENDENCIES actionlib_msgs
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  actionlib
  sensor_msgs
  # xmlrpcpp
  # unprovisioned depends.. TODO is this fine?
  Franka
)

ament_export_include_directories(include)

ament_export_libraries(franka_gripper)

add_library(franka_gripper
  src/franka_gripper.cpp
)

# add_dependencies(franka_gripper
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_generate_messages_cpp
# )

# target_link_libraries(franka_gripper
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_gripper SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )

# target_include_directories(franka_gripper PUBLIC
#   include
# )

ament_target_dependencies(franka_gripper 
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  sensor_msgs
  # xmlrpcpp
  Franka
)

add_executable(franka_gripper_node
  src/franka_gripper_node.cpp
)

# add_dependencies(franka_gripper_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   franka_gripper
# )

# target_link_libraries(franka_gripper_node
#    ${catkin_LIBRARIES}
#    franka_gripper
# )

# target_include_directories(franka_gripper_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS}
# )

ament_target_dependencies(franka_gripper_node
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  sensor_msgs
  # xmlrpcpp
  Franka
)


## Installation
# install(TARGETS franka_gripper
#                 franka_gripper_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS franka_gripper franka_gripper_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

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
  add_format_target(franka_gripper FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_gripper
    FILES ${SOURCES}
    DEPENDS franka_gripper franka_gripper_node
  )
endif()

ament_package()
