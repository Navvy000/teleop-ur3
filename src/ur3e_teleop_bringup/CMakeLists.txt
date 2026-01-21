cmake_minimum_required(VERSION 3.8)
project(ur3e_teleop_bringup)

find_package(ament_cmake REQUIRED)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch/ and config/ directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install helper scripts (Python)
install(
  PROGRAMS
    scripts/staging_pose_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
