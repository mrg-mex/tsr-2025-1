cmake_minimum_required(VERSION 3.8)
project(my_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# En esta seccion se declaran las referencias 
#necesarias
find_package(ament_cmake REQUIRED)
# rosidl_default_generators es el nombre del paquete
# donde se encuentran las herramientas para convertir 
# el archivo descriptor del mensaje (Archivo.msg) a las
# diferentes interfaces de programa (C++, Python DDS, etc.)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/MyPosition.msg" 
  "msg/TargetPosition.msg"
  "srv/SysInfo.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
