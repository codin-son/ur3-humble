cmake_minimum_required(VERSION 3.8)
project(ur_gripper_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(EXISTS meshes)
  install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
