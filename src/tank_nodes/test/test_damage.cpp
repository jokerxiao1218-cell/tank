// B 组测试:战斗结算纯逻辑(DamageCalculator)
// 用例对应设计文档第 4 节 B 组(数字全部来自需求卡)
#include <gtest/gtest.h>

#include <string>

#include "tank_nodes/damage_calculator.hpp"

using tank_nodes::DamageCalculator;

TEST(Damage, BasicApply)  // B1:25 伤打 100 血 → 剩 75;两发 25 打完死
{
  EXPECT_DOUBLE_EQ(75.0, DamageCalculator::apply(100.0, 25.0));
  EXPECT_DOUBLE_EQ(50.0, DamageCalculator::apply(75.0, 25.0));
  EXPECT_DOUBLE_EQ(25.0, DamageCalculator::apply(50.0, 25.0));
}

TEST(Damage, FireStacks)  // B2:火力层伤害 25+10*N;3 层=55;4 层封顶 60(需求卡上限)
{
  EXPECT_DOUBLE_EQ(25.0, DamageCalculator::damage_with_stacks(0));
  EXPECT_DOUBLE_EQ(35.0, DamageCalculator::damage_with_stacks(1));
  EXPECT_DOUBLE_EQ(55.0, DamageCalculator::damage_with_stacks(3));
  EXPECT_DOUBLE_EQ(60.0, DamageCalculator::damage_with_stacks(4));   // 65 被 cap 在 60
  EXPECT_DOUBLE_EQ(60.0, DamageCalculator::damage_with_stacks(99));  // 极端叠层
  EXPECT_DOUBLE_EQ(25.0, DamageCalculator::damage_with_stacks(-1));  // 负层视为 0
}

TEST(Damage, DeathClampsToZero)  // B3:第 4 发打 25 血坦克 → 恰好 0,不出负数
{
  EXPECT_DOUBLE_EQ(0.0, DamageCalculator::apply(25.0, 25.0));
  EXPECT_DOUBLE_EQ(0.0, DamageCalculator::apply(10.0, 25.0));  // 血更少也归零
}

TEST(Damage, HitThresholdEdge)  // B4:线段距离 0.49 命中、0.51 不命中(阈值 0.5)
{
  // 炮弹沿 x 轴飞过 y=0.49 处的坦克(线段水平,最近点即垂直距离)
  EXPECT_TRUE(DamageCalculator::hit(
      "bullet_p_1", "enemy_1", 0.0, 0.0, 10.0, 0.0, 5.0, 0.49));
  EXPECT_FALSE(DamageCalculator::hit(
      "bullet_p_1", "enemy_1", 0.0, 0.0, 10.0, 0.0, 5.0, 0.51));
  // 坦克在线段延长线外(炮弹已飞过):投影夹到端点
  EXPECT_TRUE(DamageCalculator::hit(
      "bullet_p_1", "enemy_1", 0.0, 0.0, 10.0, 0.0, 10.3, 0.0));   // 距端点 0.3 命中
  EXPECT_FALSE(DamageCalculator::hit(
      "bullet_p_1", "enemy_1", 0.0, 0.0, 10.0, 0.0, 10.6, 0.0));  // 距端点 0.6 不命中
  // 斜向飞行的线段同样生效
  EXPECT_TRUE(DamageCalculator::hit(
      "bullet_p_1", "enemy_1", 0.0, 0.0, 10.0, 10.0, 5.0, 5.2));
}

TEST(Damage, FriendlyFireExcluded)  // B5:同阵营不判定命中
{
  // 玩家的弹贴脸飞过玩家自己 → 不命中
  EXPECT_FALSE(DamageCalculator::hit(
      "bullet_p_1", "player", 0.0, 0.0, 10.0, 0.0, 5.0, 0.1));
  // 敌人的弹打敌人 → 不命中
  EXPECT_FALSE(DamageCalculator::hit(
      "bullet_e_1", "enemy_2", 0.0, 0.0, 10.0, 0.0, 5.0, 0.1));
  // 玩家的弹打敌人 → 命中
  EXPECT_TRUE(DamageCalculator::hit(
      "bullet_p_1", "enemy_1", 0.0, 0.0, 10.0, 0.0, 5.0, 0.1));
  // 敌人的弹打玩家 → 命中
  EXPECT_TRUE(DamageCalculator::hit(
      "bullet_e_1", "player", 0.0, 0.0, 10.0, 0.0, 5.0, 0.1));
}

TEST(Damage, DegenerateSegment)  // 边界:炮弹两拍位置重合(起点悬停)退化为点距
{
  EXPECT_TRUE(DamageCalculator::hit(
      "bullet_p_1", "enemy_1", 3.0, 3.0, 3.0, 3.0, 3.3, 3.0));
  EXPECT_FALSE(DamageCalculator::hit(
      "bullet_p_1", "enemy_1", 3.0, 3.0, 3.0, 3.0, 3.6, 3.0));
}
