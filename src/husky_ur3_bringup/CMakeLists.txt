cmake_minimum_required(VERSION 3.8)
project(husky_ur3_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config launch resources
  DESTINATION share/${PROJECT_NAME})
  
install(PROGRAMS
  husky_ur3_bringup/must3r_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME must3r_node
)
ament_package()

