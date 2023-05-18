cmake_minimum_required(VERSION 3.5)
project(adasa_description)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf worlds rviz maps routes param
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)

ament_package()
