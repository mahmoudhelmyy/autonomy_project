cmake_minimum_required(VERSION 3.8)
project(autonomy_project)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
)

ament_package()