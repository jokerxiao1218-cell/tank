// DamageCalculator:战斗结算纯逻辑(无 ROS 依赖,单测 B 组全覆盖)
//
// 命中判定采用"线段扫过"法:炮弹速度 30m/s、采样 20Hz 一拍飞 1.5m,
// 单看点距会漏弹,故用炮弹上一拍→这一拍连线与坦克中心的距离判定。
// 2D 判定(x-y 平面):炮口高、坦克被弹面高,垂直差异忽略——
// 重力下坠不影响水平命中窗口(详见设计文档 3.5 节决策 2)
#ifndef TANK_NODES__DAMAGE_CALCULATOR_HPP_
#define TANK_NODES__DAMAGE_CALCULATOR_HPP_

#include <algorithm>
#include <cmath>
#include <string>

namespace tank_nodes
{

class DamageCalculator
{
public:
  // ---- 数值(需求卡;batch 7 道具从这里乘出) ----
  static constexpr double kBaseDamage = 25.0;      // 炮弹基础伤害
  static constexpr int kMaxDamageStacks = 3;       // 火力道具叠加上限(25+10*3=55;封顶 60 由下面 cap 保证)
  static constexpr double kDamagePerStack = 10.0;
  static constexpr double kDamageCap = 60.0;       // 需求卡:伤害叠加上限 60
  static constexpr double kHitRadius = 0.5;         // 命中阈值 m(线段-坦克中心距离)
  static constexpr double kBulletSpeed = 30.0;     // 炮弹初速 m/s
  static constexpr double kBulletLifetime = 3.0;    // 炮弹寿命 s(超时自毁)

  // 伤害结算:血量只会减到 0,不会出现负数(需求卡 B3)
  static double apply(double health, double damage)
  {
    return std::max(0.0, health - damage);
  }

  // 火力道具叠加后的伤害:每层 +10,封顶 60(需求卡 B2/C2)
  static double damage_with_stacks(int stacks)
  {
    const int s = std::max(0, stacks);
    return std::min(kDamageCap, kBaseDamage + kDamagePerStack * s);
  }

  // 阵营判断:炮弹名前缀 → 阵营。玩家弹 bullet_p_,敌人弹 bullet_e_<n>
  // 同阵营不吃自己炮弹的伤(B5:玩家弹擦过玩家坦克不判定命中)
  static bool same_camp(const std::string & bullet_name, const std::string & tank_name)
  {
    const bool bullet_from_player = bullet_name.rfind("bullet_p_", 0) == 0;
    const bool target_is_player = (tank_name == "player");
    const bool bullet_from_enemy = bullet_name.rfind("bullet_e_", 0) == 0;
    const bool target_is_enemy = tank_name.rfind("enemy_", 0) == 0;
    return (bullet_from_player && target_is_player) ||
           (bullet_from_enemy && target_is_enemy);
  }

  // 点到线段的最近距离(2D)。p0→p1 为炮弹两拍连线,q 为坦克中心
  static double segment_point_distance(
    double x0, double y0, double x1, double y1,
    double px, double py)
  {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-12) {  // 退化为点
      return std::hypot(px - x0, py - y0);
    }
    // 投影参数 t 夹到 [0,1]
    const double t = std::max(0.0, std::min(
        1.0, ((px - x0) * dx + (py - y0) * dy) / len2));
    const double cx = x0 + t * dx;
    const double cy = y0 + t * dy;
    return std::hypot(px - cx, py - cy);
  }

  // 命中判定:非同阵营 + 线段距离 < 阈值
  static bool hit(
    const std::string & bullet_name, const std::string & tank_name,
    double x0, double y0, double x1, double y1,
    double tx, double ty, double radius = kHitRadius)
  {
    if (same_camp(bullet_name, tank_name)) {return false;}
    return segment_point_distance(x0, y0, x1, y1, tx, ty) < radius;
  }
};

}  // namespace tank_nodes

#endif  // TANK_NODES__DAMAGE_CALCULATOR_HPP_
