cmake_minimum_required(VERSION 3.8)
project(wallens_safety)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)    # el freno corre en la Pi: optimizado salvo que se pida otra cosa
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # sin FMA fusionado (aarch64 lo haría por defecto): mismos redondeos que Python en cada operación
  add_compile_options(-Wall -Wextra -ffp-contract=off)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Freno + vigilante en C++ (freno_lidar). Los scripts Python siguen instalados para volver atrás (launch freno:=py).
add_executable(freno_lidar src/freno_lidar.cpp)
target_include_directories(freno_lidar PRIVATE include)
ament_target_dependencies(freno_lidar rclcpp geometry_msgs sensor_msgs std_msgs)
install(TARGETS freno_lidar DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/freno_lidar.py scripts/freno_logica.py scripts/vigilante_lidar.py scripts/vigilante_logica.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_freno test/test_freno.py)
  endif()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_freno_cpp test/test_freno_cpp.cpp)
  target_include_directories(test_freno_cpp PRIVATE include)
  target_compile_definitions(test_freno_cpp PRIVATE CASOS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
endif()

ament_package()
