cmake_minimum_required(VERSION 3.5)
project(franka_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(franka_hw REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Franka 0.7.0 REQUIRED)

ament_export_dependencies(
  controller_interface
  franka_hw
  franka_msgs
  geometry_msgs
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  tf2
  tf2_msgs
  std_srvs
  Franka
)

ament_export_include_directories(include)
ament_export_libraries(franka_state_controller)

## franka_state_controller
add_library(franka_state_controller
  src/franka_state_controller.cpp
)

# add_dependencies(franka_state_controller
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(franka_state_controller
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_state_controller SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )
# target_include_directories(franka_state_controller PUBLIC
#   include
# )

ament_target_dependencies(franka_state_controller
  controller_interface
  franka_hw
  franka_msgs
  geometry_msgs
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  tf2
  tf2_msgs
  std_srvs
  Franka
)


## franka_control_node
add_executable(franka_control_node
  src/franka_control_node.cpp
)

# add_dependencies(franka_control_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(franka_control_node
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_control_node SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )

add_executable(franka_combined_control_node
    src/franka_combined_control_node.cpp
)

# add_dependencies(franka_combined_control_node
#     ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(franka_combined_control_node
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_combined_control_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS}
# )

ament_target_dependencies(franka_combined_control_node
  controller_interface
  franka_hw
  franka_msgs
  geometry_msgs
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  tf2_msgs
  std_srvs
  Franka
)


## Installation
install(TARGETS franka_state_controller
                franka_control_node
                franka_combined_control_node
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
install(FILES franka_controller_plugins.xml
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
  add_format_target(franka_control FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_control
    FILES ${SOURCES}
    DEPENDS franka_control_node franka_combined_control_node franka_state_controller
  )
endif()

ament_package()
