cmake_minimum_required(VERSION 3.8)
project(HexNav_description)

find_package(ament_cmake REQUIRED)

# Install package data
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable Python scripts (joint control GUI, etc.)
install(PROGRAMS
  scripts/joint_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
