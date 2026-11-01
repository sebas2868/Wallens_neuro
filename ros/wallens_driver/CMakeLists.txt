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

add_executable(wallens_driver src/wallens_driver.cpp)
target_compile_features(wallens_driver PUBLIC cxx_std_17)
ament_target_dependencies(wallens_driver rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros dynamixel_sdk)

install(TARGETS wallens_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
