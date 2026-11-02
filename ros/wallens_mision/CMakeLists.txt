cmake_minimum_required(VERSION 3.8)
project(wallens_mision)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # el detector procesa video: sin optimizar gasta mucho más CPU
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)

add_executable(detector_frutas src/detector_frutas.cpp)
target_compile_features(detector_frutas PUBLIC cxx_std_17)
target_link_libraries(detector_frutas ${OpenCV_LIBS})
ament_target_dependencies(detector_frutas rclcpp std_msgs)
install(TARGETS detector_frutas DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/mision_frutas.py scripts/mision_logica.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_mision test/test_mision.py)
  endif()
endif()

ament_package()
