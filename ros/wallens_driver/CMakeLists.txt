cmake_minimum_required(VERSION 3.8)
project(wallens_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(wallens_interfaces REQUIRED)

add_executable(wallens_driver src/wallens_driver.cpp)
target_compile_features(wallens_driver PUBLIC cxx_std_17)
target_include_directories(wallens_driver PRIVATE include)
ament_target_dependencies(wallens_driver rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros dynamixel_sdk wallens_interfaces)

install(TARGETS wallens_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Pruebas de BusBrazo con un puerto falso (sin SDK ni ROS): colcon test --packages-select wallens_driver
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bus_brazo test/test_bus_brazo.cpp)
  target_include_directories(test_bus_brazo PRIVATE include)
  target_compile_features(test_bus_brazo PUBLIC cxx_std_17)
endif()

ament_package()
