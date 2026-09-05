// D 组测试:敌人 AI 决策纯逻辑(EnemyDecision)
// 用例对应设计文档第 4 节 D 组
#include <gtest/gtest.h>

#include "tank_nodes/enemy_decision.hpp"
#include "tank_msgs/msg/game_state.hpp"

using tank_nodes::EnemyDecision;
using GS = tank_msgs::msg::GameState;

TEST(Enemy, PatrolWhenFar)  // D1:玩家 25m → 巡逻:朝巡逻点开、不开炮
{
  // 敌人 (0,0,朝x+),巡逻点 (3,0),玩家 (25,0)
  auto o = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 25, 0, 3, 0, 100.0, -100.0);
  EXPECT_GT(o.linear, 0.0);   // 朝巡逻点前进
  EXPECT_NEAR(o.angular, 0.0, 1e-9);
  EXPECT_FALSE(o.fire);
  EXPECT_FALSE(o.arrived_patrol);
}

TEST(Enemy, ChaseAtRangeEdge)  // D2:玩家 19m(阈值 20 内侧)→ 追击:朝玩家开
{
  // 敌人 (0,0,朝x+),玩家在 (19,0) 正前方 → 直追
  auto o = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 19, 0, 3, 0, 100.0, -100.0);
  EXPECT_GT(o.linear, 0.0);
  EXPECT_NEAR(o.angular, 0.0, 1e-9);
  EXPECT_NEAR(o.turret_joint, 0.0, 1e-9);  // 炮口指玩家(正前)

  // 玩家在侧后方 (0,19) → 先转向(大角度不前进)
  auto o2 = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 0, 19, 3, 0, 100.0, -100.0);
  EXPECT_DOUBLE_EQ(0.0, o2.linear);
  EXPECT_GT(o2.angular, 0.0);  // 左转朝玩家
  EXPECT_NEAR(o2.turret_joint, M_PI / 2, 1e-9);  // 炮塔关节指右方玩家
}

TEST(Enemy, FireInRangeWithCooldown)  // D3:14m 开炮;冷却 2s 内第二拍不开
{
  const double now = 100.0;
  auto o1 = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 14, 0, 3, 0, now, -100.0);
  EXPECT_TRUE(o1.fire);   // 冷却已过 → 开炮
  auto o2 = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 14, 0, 3, 0, now + 1.0, now);
  EXPECT_FALSE(o2.fire);  // 距上次开炮 1s < 2s → 不开
  auto o3 = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 14, 0, 3, 0, now + 2.0, now);
  EXPECT_TRUE(o3.fire);  // 冷却刚满 → 开
  // 16m(15 开炮阈值外)追击但不开炮
  auto o4 = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 16, 0, 3, 0, now, -100.0);
  EXPECT_FALSE(o4.fire);
  EXPECT_GT(o4.linear, 0.0);
}

TEST(Enemy, PausedFrozen)  // D4:PAUSED 下任何距离 → 不动不打
{
  auto o = EnemyDecision::decide(GS::PAUSED, 0, 0, 0, 1, 0, 3, 0, 100.0, -100.0);
  EXPECT_DOUBLE_EQ(0.0, o.linear);
  EXPECT_DOUBLE_EQ(0.0, o.angular);
  EXPECT_DOUBLE_EQ(0.0, o.turret_joint);
  EXPECT_FALSE(o.fire);
  // IDLE 同样冻结
  auto o2 = EnemyDecision::decide(GS::IDLE, 0, 0, 0, 1, 0, 3, 0, 100.0, -100.0);
  EXPECT_DOUBLE_EQ(0.0, o2.linear);
  EXPECT_FALSE(o2.fire);
}

TEST(Enemy, PatrolArrivalSignal)  // 边界:距巡逻点 <1m → 上报到达(节点换目标)
{
  auto o = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 25, 0, 0.5, 0, 100.0, -100.0);
  EXPECT_TRUE(o.arrived_patrol);
}

TEST(Enemy, KeepDistanceInChase)  // D5:追击中 <5m 停车开炮(防贴脸死角)
{
  // 玩家 4m(贴脸圈):停车,但仍开炮
  auto o = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 4, 0, 3, 0, 100.0, -100.0);
  EXPECT_DOUBLE_EQ(0.0, o.linear);
  EXPECT_TRUE(o.fire);
  // 玩家 6m(圈外):继续逼近
  auto o2 = EnemyDecision::decide(GS::RUNNING, 0, 0, 0, 6, 0, 3, 0, 100.0, -100.0);
  EXPECT_GT(o2.linear, 0.0);
  EXPECT_TRUE(o2.fire);
}
