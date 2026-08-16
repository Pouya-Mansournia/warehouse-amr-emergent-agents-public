cmake_minimum_required(VERSION 3.8)
project(amr_ros_dg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf rviz worlds web
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/wasd_teleop.py scripts/web_dashboard.py scripts/fleet_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
