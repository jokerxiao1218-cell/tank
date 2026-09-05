cmake_minimum_required(VERSION 3.8)
project(tank_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tank_msgs REQUIRED)

# 节点可执行文件
add_executable(keyboard_node src/keyboard_node.cpp)
ament_target_dependencies(keyboard_node rclcpp tank_msgs geometry_msgs std_msgs)
target_include_directories(keyboard_node PRIVATE include)

add_executable(player_tank_node src/player_tank_node.cpp)
ament_target_dependencies(player_tank_node rclcpp tank_msgs std_msgs gazebo_msgs)
target_include_directories(player_tank_node PRIVATE include)

add_executable(combat_system_node src/combat_system_node.cpp)
ament_target_dependencies(combat_system_node rclcpp tank_msgs gazebo_msgs)
target_include_directories(combat_system_node PRIVATE include)

install(TARGETS keyboard_node player_tank_node combat_system_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 冒烟:验证消息代码生成正确、gtest 基建可用
  ament_add_gtest(test_smoke test/test_smoke.cpp)
  ament_target_dependencies(test_smoke tank_msgs)
  # 键盘解析纯逻辑(E 组用例);-O0 -g 保证可单步调试
  ament_add_gtest(test_keymap test/test_keymap.cpp)
  target_compile_options(test_keymap PRIVATE -O0 -g)
  target_include_directories(test_keymap PRIVATE include)
  ament_target_dependencies(test_keymap tank_msgs)
  # 战斗结算纯逻辑(B 组用例)
  ament_add_gtest(test_damage test/test_damage.cpp)
  target_compile_options(test_damage PRIVATE -O0 -g)
  target_include_directories(test_damage PRIVATE include)
endif()

ament_package()
