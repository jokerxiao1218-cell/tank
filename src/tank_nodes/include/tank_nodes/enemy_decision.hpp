// EnemyDecision:敌人 AI 决策纯逻辑(无 ROS 依赖,单测 D 组覆盖)
//
// 规则(需求卡数值):
//   玩家距离 > 20m → 巡逻(朝巡逻目标点开)
//   玩家距离 ≤ 20m → 追击(朝玩家开)
//   玩家距离 ≤ 15m → 开炮(冷却 2s)
//   游戏状态非 RUNNING → 一切归零(D4:暂停冻结)
// 决策是无状态的:开炮冷却由调用方传入上次开炮时刻(便于单测)
#ifndef TANK_NODES__ENEMY_DECISION_HPP_
#define TANK_NODES__ENEMY_DECISION_HPP_

#include <cmath>
#include <cstdint>

#include "tank_msgs/msg/game_state.hpp"

namespace tank_nodes
{

class EnemyDecision
{
public:
  static constexpr double kChaseRange = 20.0;   // 追击触发距离 m
  static constexpr double kFireRange = 15.0;    // 开炮距离 m
  static constexpr double kKeepDistance = 5.0;   // 保持交战距离 m(更近则停车开炮)
  static constexpr double kFireCooldown = 2.0;  // 开炮冷却 s
  static constexpr double kMaxLinear = 1.0;     // m/s
  static constexpr double kMaxAngular = 1.0;    // rad/s
  static constexpr double kArriveRadius = 1.0;  // 巡逻点到达半径 m

  struct Output
  {
    double linear = 0.0;
    double angular = 0.0;
    double turret_joint = 0.0;  // 炮塔关节角(相对车体,执行器语义)
    bool fire = false;
    bool arrived_patrol = false;  // 到达巡逻点(节点层据此换目标)
  };

  // 全部输入由调用方提供:
  //   ex,ey,eyaw:自身位姿(eyaw 为车体朝向,弧度)
  //   px,py:玩家位置;tx,ty:当前巡逻目标点
  //   now:当前仿真秒;last_fire:上次开炮仿真秒(未开过炮给负值)
  static Output decide(
    uint8_t game_state,
    double ex, double ey, double eyaw,
    double px, double py,
    double tx, double ty,
    double now, double last_fire)
  {
    Output o;
    if (game_state != tank_msgs::msg::GameState::RUNNING) {
      return o;  // D4:非 RUNNING 一切归零
    }

    const double dist = std::hypot(px - ex, py - ey);
    const double fire_ready = (now - last_fire) >= kFireCooldown;

    if (dist > kChaseRange) {
      // 巡逻:朝巡逻点开
      const double ang_to_target = std::atan2(ty - ey, tx - ex);
      const double diff = normalize(ang_to_target - eyaw);
      o.angular = clamp(diff, -kMaxAngular, kMaxAngular);
      o.linear = (std::fabs(diff) < 0.5) ? kMaxLinear : 0.0;  // 大角度先转向
      o.turret_joint = 0.0;  // 巡逻时炮塔回正(跟车头一致)
      o.arrived_patrol = (std::hypot(tx - ex, ty - ey) < kArriveRadius);
      return o;
    }

    // 追击:朝玩家开 + 炮塔瞄准玩家 + 距离内开炮
    // 近到 kKeepDistance 内停车开炮:继续冲会贴脸(B6 实测冲到 0.01m,
    // 炮口前伸 1.2m 越过玩家,炮弹生成在玩家身后 → 反而永远打不中)
    const double ang_to_player = std::atan2(py - ey, px - ex);
    const double diff = normalize(ang_to_player - eyaw);
    o.angular = clamp(diff, -kMaxAngular, kMaxAngular);
    o.linear = (std::fabs(diff) < 0.5 && dist > kKeepDistance) ? kMaxLinear : 0.0;
    // 炮塔关节角 = 世界瞄准角 - 车体朝向(相对角,执行器吃绝对关节目标)
    o.turret_joint = normalize(ang_to_player - eyaw);
    if (dist <= kFireRange && fire_ready) {
      o.fire = true;
    }
    return o;
  }

private:
  static double normalize(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }
  static double clamp(double v, double lo, double hi)
  {
    return v < lo ? lo : (v > hi ? hi : v);
  }
};

}  // namespace tank_nodes

#endif  // TANK_NODES__ENEMY_DECISION_HPP_
