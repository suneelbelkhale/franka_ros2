cmake_minimum_required(VERSION 3.5)
project(franka_description)

find_package(ament_cmake REQUIRED)
ament_export_dependencies(xacro)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()