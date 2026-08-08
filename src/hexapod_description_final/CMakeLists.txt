cmake_minimum_required(VERSION 3.8)
project(hexapod_description_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Compilation du noeud C++ de demarche
add_executable(hexapod_gait_node src/hexapod_gait_node.cpp)
ament_target_dependencies(hexapod_gait_node rclcpp sensor_msgs)

install(
  DIRECTORY config launch meshes urdf textures
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  hexapod_gait_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()