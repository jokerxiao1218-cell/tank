cmake_minimum_required(VERSION 3.8)
project(tank_msgs)

# rosidl:从 .msg 文件生成 C++/Python 代码的官方工具链
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GameState.msg"
  "msg/TankStatus.msg"
  "msg/PowerupEvent.msg"
)

ament_package()
