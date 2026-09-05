cmake_minimum_required(VERSION 3.8)
project(tank_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds params
  DESTINATION share/${PROJECT_NAME})

ament_package()
