cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

# Install meshes, urdf, and launch folders
# Install the mesh directories
install(DIRECTORY
  dae
  meshes
  urdf
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
