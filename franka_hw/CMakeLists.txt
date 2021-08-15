cmake_minimum_required(VERSION 3.5)
project(franka_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(actionlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(combined_robot_hw REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

ament_export_dependencies(actionlib_msgs
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
    # unprovisioned depends.. TODO is this fine?
    Franka
)
ament_export_include_directories(include)
ament_export_libraries(franka_hw franka_control_services)


add_library(franka_hw
  src/control_mode.cpp
  src/franka_hw.cpp
  src/franka_combinable_hw.cpp
  src/franka_combined_hw.cpp
  src/resource_helpers.cpp
  src/trigger_rate.cpp
)

# add_dependencies(franka_hw
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# I think this is necessary... for Franka?
# target_link_libraries(franka_hw
#   ${Franka_LIBRARIES}
# )

# target_include_directories(franka_hw SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
# )

ament_target_dependencies(franka_hw 
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


# target_include_directories(franka_hw SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )
# target_include_directories(franka_hw PUBLIC
#   include
# )

## franka_control_services
add_library(franka_control_services
  src/services.cpp
)

ament_target_dependencies(franka_control_services rclcpp franka_msgs Franka)

# add_dependencies(franka_control_services
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(franka_control_services
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_control_services SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )

# target_include_directories(franka_control_services PUBLIC
#   include
# )

# TODO
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

## Installation
# install(TARGETS franka_hw franka_control_services
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS franka_hw franka_control_services
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
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
  add_format_target(franka_hw FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_hw
    FILES ${SOURCES}
    DEPENDS franka_hw
  )
endif()

ament_package()
