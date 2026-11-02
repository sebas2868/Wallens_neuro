cmake_minimum_required(VERSION 3.8)
project(wallens_power)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Componente (cargable en el contenedor wallens_base) y, con el mismo nombre, el ejecutable suelto
add_library(wallens_power_componente SHARED src/wallens_power.cpp)
target_compile_features(wallens_power_componente PUBLIC cxx_std_17)
ament_target_dependencies(wallens_power_componente rclcpp rclcpp_components sensor_msgs std_msgs)
rclcpp_components_register_node(wallens_power_componente PLUGIN "wallens_power::WallensPower" EXECUTABLE wallens_power)
install(TARGETS wallens_power_componente ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
