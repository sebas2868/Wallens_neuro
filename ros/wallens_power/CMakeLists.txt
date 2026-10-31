cmake_minimum_required(VERSION 3.8)
project(wallens_power)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(wallens_power src/wallens_power.cpp)
target_compile_features(wallens_power PUBLIC cxx_std_17)
ament_target_dependencies(wallens_power rclcpp sensor_msgs std_msgs)

install(TARGETS wallens_power DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
