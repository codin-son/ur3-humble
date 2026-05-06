cmake_minimum_required(VERSION 3.8)
project(ur_hande_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "*.noetic.bak" EXCLUDE
  PATTERN "setup_assistant*" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
