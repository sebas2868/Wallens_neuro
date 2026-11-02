cmake_minimum_required(VERSION 3.8)
project(wallens_mision)

find_package(ament_cmake REQUIRED)

# Fase 2: solo la lógica (sin ROS). El nodo mision_frutas.py y los detectores llegan en las fases siguientes.
install(PROGRAMS scripts/mision_logica.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_mision test/test_mision.py)
  endif()
endif()

ament_package()
