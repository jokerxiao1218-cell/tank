cmake_minimum_required(VERSION 3.8)
project(tank_description)

find_package(ament_cmake REQUIRED)

# 模型资源按目录装到 share 下,launch 里用 $(find-pkg-share tank_description) 引用
install(DIRECTORY urdf meshes sdf controllers
  DESTINATION share/${PROJECT_NAME})

ament_package()
