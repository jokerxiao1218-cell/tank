// GameStateMachine:游戏总控状态机纯逻辑(无 ROS 依赖,单测 A 组全覆盖)
//
// 状态:IDLE →(start)→ RUNNING ↔(pause 切换)→ PAUSED
//       RUNNING/PAUSED → WIN(敌人全灭) / LOSE(玩家被击毁)
// 终局(WIN/LOSE)不可再 start(A7)
//
// 胜负输入是异步到达的(combat 在命中结算后才发),状态机只管转换规则;
// "PAUSED 下胜负是否生效"定为生效(A6):暂停前的最后一炮要结算
#ifndef TANK_NODES__GAME_STATE_MACHINE_HPP_
#define TANK_NODES__GAME_STATE_MACHINE_HPP_

#include <cstdint>
#include <string>
#include <utility>

#include "tank_msgs/msg/game_state.hpp"

namespace tank_nodes
{

class GameStateMachine
{
public:
  struct Result
  {
    bool ok = false;
    std::string message;
  };

  // start:仅 IDLE 时有效
  Result handle_start()
  {
    switch (state_) {
      case tank_msgs::msg::GameState::IDLE:
        state_ = tank_msgs::msg::GameState::RUNNING;
        return {true, "游戏开始"};
      case tank_msgs::msg::GameState::RUNNING:
      case tank_msgs::msg::GameState::PAUSED:
        return {false, "游戏已在进行中"};
      default:  // WIN / LOSE
        return {false, "本局已结束,不可重新开始"};
    }
  }

  // pause 切换:RUNNING→PAUSED、PAUSED→RUNNING;IDLE 无效(A3)
  Result handle_pause_toggle()
  {
    switch (state_) {
      case tank_msgs::msg::GameState::RUNNING:
        state_ = tank_msgs::msg::GameState::PAUSED;
        return {true, "已暂停"};
      case tank_msgs::msg::GameState::PAUSED:
        state_ = tank_msgs::msg::GameState::RUNNING;
        return {true, "已恢复"};
      default:
        return {false, "游戏未开始,无法暂停"};
    }
  }

  // 敌人存活数输入(节点层保证只在"见过至少一个敌人状态"后调用,
  // 防止启动初期 status 未到时误判全灭)
  void on_enemy_alive_count(int n)
  {
    enemy_alive_ = n;
    if (n == 0) {
      transition_to_end(tank_msgs::msg::GameState::WIN);
    }
  }

  // 玩家存活输入
  void on_player_alive(bool alive)
  {
    if (!alive) {
      transition_to_end(tank_msgs::msg::GameState::LOSE);
    }
  }

  uint8_t state() const {return state_;}
  int enemy_alive() const {return enemy_alive_;}

private:
  // 终局转换:RUNNING/PAUSED 下生效;IDLE(还没开打)和已终局的不动
  void transition_to_end(uint8_t end_state)
  {
    if (state_ == tank_msgs::msg::GameState::RUNNING ||
      state_ == tank_msgs::msg::GameState::PAUSED)
    {
      state_ = end_state;
    }
  }

  uint8_t state_ = tank_msgs::msg::GameState::IDLE;
  int enemy_alive_ = 0;
};

}  // namespace tank_nodes

#endif  // TANK_NODES__GAME_STATE_MACHINE_HPP_
