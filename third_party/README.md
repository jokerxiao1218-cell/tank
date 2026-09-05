# third_party 说明:第三方模型来源

## zthanxx_tankbot_description/

- **来源**:https://github.com/zthanxx/tank_bot (zthanxx fork 自 Toshinori Kitamura 的日本开源履带坦克项目)
- **协议**:MIT License(见 zthanxx_tankbot_description/LICENSE,Copyright (c) 2018 Toshinori Kitamura)
- **用途**:本项目(tank-battle)的坦克底盘模型,取其 urdf/ 与 meshes/ 履带机器人部分;炮塔炮管为本项目自加的几何体
- **改动**:
  - 删除了与游戏无关的 maps/(65MB 建图数据)、dwa/、world/、launch/ 目录
  - urdf 中 Gazebo 插件标签后续将按 ROS2 Humble gazebo_ros_pkgs 格式重写(原为 ROS1 格式)
  - 模型整体尺寸将按游戏需要放大(原约 30cm 玩具尺寸)
- **取用时间**:2026-09-05,经"真实 clone + 文件级验证"(11 link / 10 joint,STL 全齐,原始 252KB)
