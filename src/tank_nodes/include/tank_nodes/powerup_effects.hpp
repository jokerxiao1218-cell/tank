// PowerupEffects:道具效果纯逻辑(无 ROS 依赖,单测 C 组覆盖)
//
// 4 种道具(需求卡数值):
//   rapid_fire  射速:开炮冷却 ×0.5(1.5s→0.75s)
//   damage_up   火力:伤害叠层 +1(25+10×层,4 层封顶 60)
//   speed_up    速度:底盘速度倍率 +0.3(封顶 1.9)
//   repair      回血:血量回满(不超上限)
// 场上同时最多 5 个道具(PowerupManager 保证,这里提供上限常量)
#ifndef TANK_NODES__POWERUP_EFFECTS_HPP_
#define TANK_NODES__POWERUP_EFFECTS_HPP_

#include "tank_nodes/damage_calculator.hpp"

namespace tank_nodes
{

class PowerupEffects
{
public:
  static constexpr int kMaxOnField = 5;      // 场上同时存在的道具上限
  static constexpr double kRapidFireScale = 0.5;   // 射速道具:冷却倍率
  static constexpr int kMaxDamageStacksCap = 4;      // 火力叠层上限(60 封顶对应 4 层)
  static constexpr double kSpeedPerStack = 0.3;     // 每层速度倍率增量
  static constexpr double kMaxSpeedScale = 1.9;      // 速度倍率封顶
  static constexpr double kDefaultCooldown = 1.5;    // 默认开炮冷却 s
  static constexpr double kDefaultSpeedScale = 1.0;  // 默认底盘速度倍率

  // ---- 各 buff 生效(输入当前值,输出新值;全部幂等封顶) ----

  // 射速:叠一层 ×0.5?需求卡是"射速 1.5→0.75"即一次性的半冷却,
  // 再叠不更狠(保持 0.75;避免 0.375s 冷却让游戏失衡)
  static double apply_rapid_fire(double cooldown)
  {
    return cooldown * kRapidFireScale;
  }

  // 火力:层数 +1,伤害 = 25+10×层,封顶 60(4 层)
  static double apply_damage_up(int stacks)
  {
    const int s = stacks > kMaxDamageStacksCap ? kMaxDamageStacksCap : stacks;
    return DamageCalculator::damage_with_stacks(s);
  }

  // 速度:倍率 +0.3,封顶 1.9
  static double apply_speed_up(double scale)
  {
    const double v = scale + kSpeedPerStack;
    return v > kMaxSpeedScale ? kMaxSpeedScale : v;
  }

  // 回血:直接回满(血量只会 ≤ 上限,需求卡"维修回满不超")
  static double apply_repair(double /*health*/, double max_health)
  {
    return max_health;
  }
};

}  // namespace tank_nodes

#endif  // TANK_NODES__POWERUP_EFFECTS_HPP_
