// KeymapParser:tank-battle 键盘解析纯逻辑(无任何 ROS 依赖,单测全覆盖)
//
// 键位语义(teleop 风格,ROS 生态通行做法):
//   W/S 前进/后退(按一下持续)  A/D 左/右转向   X 急停(底盘全停)
//   Q/E 炮塔左/右步进 0.2rad     空格 开炮     Enter 开始    P 暂停/恢复
// 为什么不是"按住持续/松开停":Linux 终端按键只有按下事件、没有松开事件,
// 需求卡的"按住持续"语义无法实现,详见设计文档测试计划 E 组变更记录。
#ifndef TANK_NODES__KEYMAP_PARSER_HPP_
#define TANK_NODES__KEYMAP_PARSER_HPP_

#include <cstdint>
#include <utility>

#include "tank_msgs/msg/game_state.hpp"

namespace tank_nodes
{

class KeymapParser
{
public:
  // 数值集中在此(需求卡数值;速度道具 batch 7 在节点层做乘法)
  static constexpr double kMaxSpeed = 1.0;    // m/s
  static constexpr double kMaxTurnRate = 1.0;  // rad/s
  static constexpr double kTurretStep = 0.2;   // rad

  explicit KeymapParser(uint8_t initial_state = tank_msgs::msg::GameState::RUNNING)
  : game_state_(initial_state) {}

  void set_game_state(uint8_t state) { game_state_ = state; }
  uint8_t game_state() const { return game_state_; }

  // 处理一次按键事件。
  // 锁死规则:非 RUNNING 时运动/炮塔/开炮键一律忽略;
  // Enter(开始)与 P(暂停)属于游戏控制键,任何状态都放行——
  // 否则 IDLE 状态下永远无法开始游戏。
  void on_key(char c)
  {
    if (c == '\n' || c == '\r') {start_requested_ = true; return;}
    if (c == 'p') {pause_requested_ = true; return;}
    if (c == ' ') {
      if (game_state_ == tank_msgs::msg::GameState::RUNNING) {fire_requested_ = true;}
      return;
    }
    if (game_state_ != tank_msgs::msg::GameState::RUNNING) {return;}

    switch (c) {
      case 'w': linear_ = kMaxSpeed; break;
      case 's': linear_ = -kMaxSpeed; break;
      case 'a': angular_ = kMaxTurnRate; break;    // ROS 约定:z>0 逆时针(左)
      case 'd': angular_ = -kMaxTurnRate; break;
      case 'x': linear_ = 0.0; angular_ = 0.0; break;
      case 'q': turret_target_ += kTurretStep; break;  // 炮塔向左(逆时针)
      case 'e': turret_target_ -= kTurretStep; break;  // 炮塔向右(顺时针)
      default: break;  // 未知键忽略,不影响任何状态
    }
  }

  // 运动输出(非 RUNNING 时强制为 0,双保险:即使 on_key 漏拦,这里也锁死)
  double linear() const
  {
    return game_state_ == tank_msgs::msg::GameState::RUNNING ? linear_ : 0.0;
  }
  double angular() const
  {
    return game_state_ == tank_msgs::msg::GameState::RUNNING ? angular_ : 0.0;
  }
  // 炮塔绝对目标角(rad),由 Q/E 步进累积;连续关节,累积无界
  double turret_target() const {return turret_target_;}

  // 一次性事件:取走后自动清零(pop 语义),节点层据此发对应消息
  bool pop_start_requested() {return std::exchange(start_requested_, false);}
  bool pop_pause_requested() {return std::exchange(pause_requested_, false);}
  bool pop_fire_requested() {return std::exchange(fire_requested_, false);}

private:
  uint8_t game_state_;
  double linear_ = 0.0;
  double angular_ = 0.0;
  double turret_target_ = 0.0;
  bool start_requested_ = false;
  bool pause_requested_ = false;
  bool fire_requested_ = false;
};

}  // namespace tank_nodes

#endif  // TANK_NODES__KEYMAP_PARSER_HPP_
