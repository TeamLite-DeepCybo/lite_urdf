cmake_minimum_required(VERSION 3.8)
project(lite_urdf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf mappings config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
