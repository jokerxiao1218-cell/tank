#!/bin/bash
# tank-battle 全量清理:杀掉所有仿真相关进程。
# 教训(B6):孤儿节点会污染 DDS 图(多提供者抢答服务、服务调用卡死),
# 每次 launch 前后都要跑一遍。脚本名含 tank_clean,ps 模式里排除自身防自杀。
ps aux | grep -E "game_master_node|keyboard_node|player_tank_node|combat_system_node|enemy_ai_node|gzserver|gzclient|robot_state_publisher|controller_manager|spawn_entity|ros2 launch|ros2 run|ros2 topic|ros2 service|ros2 param" \
  | grep -v grep | grep -v tank_clean | awk '{print $2}' | xargs -r kill -9 2>/dev/null
sleep 1
N=$(ps aux | grep -E "game_master_node|gzserver|robot_state_publisher" | grep -v grep | grep -v tank_clean | wc -l)
echo "清理完成,残留 $N 个"
exit 0
