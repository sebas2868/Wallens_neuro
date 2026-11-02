cmake_minimum_required(VERSION 3.8)
project(wallens_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OperacionDxl.msg"
  "msg/ResultadoDxl.msg"
  "msg/DiagnosticoBus.msg"
  "msg/EstadoBrazo.msg"
  "srv/TransaccionDxl.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
