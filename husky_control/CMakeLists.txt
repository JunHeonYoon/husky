cmake_minimum_required(VERSION 3.5)
project(husky_control)


find_package(ament_cmake REQUIRED)


install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
