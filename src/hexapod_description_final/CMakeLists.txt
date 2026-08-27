cmake_minimum_required(VERSION 3.8)
project(hexapod_description_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Compilation du nœud C++
add_executable(my_hexapod_node src/myHexapod.cpp)
ament_target_dependencies(my_hexapod_node rclcpp sensor_msgs)

# Installation de l'exécutable et des dossiers du package
install(TARGETS my_hexapod_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
