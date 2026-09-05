// 冒烟测试:验证 tank_msgs 消息代码生成正确(常量、字段默认值)
// batch 1 建立 gtest 基建;后续各 batch 的逻辑类测试在此基础上追加
#include <gtest/gtest.h>

#include "tank_msgs/msg/game_state.hpp"
#include "tank_msgs/msg/tank_status.hpp"
#include "tank_msgs/msg/powerup_event.hpp"

TEST(Smoke, GameStateConstants) {
  // 常量必须与设计文档 3.3 节/消息定义一致
  EXPECT_EQ(0u, tank_msgs::msg::GameState::IDLE);
  EXPECT_EQ(1u, tank_msgs::msg::GameState::RUNNING);
  EXPECT_EQ(2u, tank_msgs::msg::GameState::PAUSED);
  EXPECT_EQ(3u, tank_msgs::msg::GameState::WIN);
  EXPECT_EQ(4u, tank_msgs::msg::GameState::LOSE);
}

TEST(Smoke, DefaultValues) {
  auto gs = tank_msgs::msg::GameState();
  EXPECT_EQ(gs.state, tank_msgs::msg::GameState::IDLE);
  EXPECT_EQ(gs.enemy_alive, 0u);

  auto ts = tank_msgs::msg::TankStatus();
  EXPECT_TRUE(ts.tank_name.empty());
  EXPECT_DOUBLE_EQ(ts.health, 0.0);
  EXPECT_FALSE(ts.is_player);
  EXPECT_FALSE(ts.alive);

  auto pe = tank_msgs::msg::PowerupEvent();
  EXPECT_TRUE(pe.powerup_type.empty());
  EXPECT_TRUE(pe.collector.empty());
}
