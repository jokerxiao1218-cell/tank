cmake_minimum_required(VERSION 3.8)
project(tank_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tank_msgs REQUIRED)

# 后续 batch 在此追加节点可执行文件,统一模式:
# add_executable(<节点名> src/<节点名>.cpp)
# ament_target_dependencies(<节点名> rclcpp tank_msgs ...)
# install(TARGETS <节点名> DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 冒烟:验证消息代码生成正确、gtest 基建可用
  ament_add_gtest(test_smoke test/test_smoke.cpp)
  ament_target_dependencies(test_smoke tank_msgs)
endif()

ament_package()
