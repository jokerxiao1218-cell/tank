// C 组测试:道具效果纯逻辑(PowerupEffects)
// 用例对应设计文档第 4 节 C 组
#include <gtest/gtest.h>

#include "tank_nodes/powerup_effects.hpp"

using tank_nodes::PowerupEffects;

TEST(Powerup, RapidFireHalvesCooldown)  // C1:射速道具 1.5s→0.75s
{
  EXPECT_DOUBLE_EQ(0.75, PowerupEffects::apply_rapid_fire(1.5));
}

TEST(Powerup, DamageStacksWithCap)  // C2:火力 25/35/45/55,4 层封顶 60
{
  EXPECT_DOUBLE_EQ(25.0, PowerupEffects::apply_damage_up(0));
  EXPECT_DOUBLE_EQ(35.0, PowerupEffects::apply_damage_up(1));
  EXPECT_DOUBLE_EQ(45.0, PowerupEffects::apply_damage_up(2));
  EXPECT_DOUBLE_EQ(55.0, PowerupEffects::apply_damage_up(3));
  EXPECT_DOUBLE_EQ(60.0, PowerupEffects::apply_damage_up(4));   // 4 层 = 封顶
  EXPECT_DOUBLE_EQ(60.0, PowerupEffects::apply_damage_up(99));  // 溢出仍封顶 60
}

TEST(Powerup, SpeedStacksCapped)  // C3:速度 1.0→1.3→1.6→1.9→封顶
{
  double s = PowerupEffects::kDefaultSpeedScale;
  s = PowerupEffects::apply_speed_up(s);
  EXPECT_DOUBLE_EQ(1.3, s);
  s = PowerupEffects::apply_speed_up(s);
  EXPECT_DOUBLE_EQ(1.6, s);
  s = PowerupEffects::apply_speed_up(s);
  EXPECT_DOUBLE_EQ(1.9, s);
  s = PowerupEffects::apply_speed_up(s);  // 封顶不再涨
  EXPECT_DOUBLE_EQ(1.9, s);
}

TEST(Powerup, RepairFillsToMax)  // C4:维修回满、不超上限
{
  EXPECT_DOUBLE_EQ(100.0, PowerupEffects::apply_repair(20.0, 100.0));
  EXPECT_DOUBLE_EQ(100.0, PowerupEffects::apply_repair(99.9, 100.0));
}

TEST(Powerup, FieldLimitConstant)  // C5:场上道具上限 5(管理器按此截断)
{
  EXPECT_EQ(5, PowerupEffects::kMaxOnField);
}

TEST(Powerup, RapidFireIdempotentish)  // C6:连吃两颗射速,不失控(≥0.3s)
{
  const double one = PowerupEffects::apply_rapid_fire(1.5);
  const double two = PowerupEffects::apply_rapid_fire(one);
  EXPECT_DOUBLE_EQ(0.375, two);
  EXPECT_GT(two, 0.3);  // 冷却不归零(游戏不失衡的底线)
}
