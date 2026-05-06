cmake_minimum_required(VERSION 3.8)
project(ur_handeye_calibration)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "*.noetic.bak" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/calibrate.py
  scripts/calibrator.py
  scripts/capture_camera_poses.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
