// A 组测试:游戏状态机纯逻辑(GameStateMachine)
// 用例对应设计文档第 4 节 A 组
#include <gtest/gtest.h>

#include "tank_nodes/game_state_machine.hpp"
#include "tank_msgs/msg/game_state.hpp"

using tank_nodes::GameStateMachine;
using GS = tank_msgs::msg::GameState;

TEST(GameState, StartFromIdle)  // A1:初始 IDLE;start → RUNNING
{
  GameStateMachine m;
  EXPECT_EQ(GS::IDLE, m.state());
  auto r = m.handle_start();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(GS::RUNNING, m.state());
}

TEST(GameState, PauseToggle)  // A2:RUNNING pause→PAUSED;再 pause→RUNNING
{
  GameStateMachine m;
  m.handle_start();
  EXPECT_TRUE(m.handle_pause_toggle().ok);
  EXPECT_EQ(GS::PAUSED, m.state());
  EXPECT_TRUE(m.handle_pause_toggle().ok);
  EXPECT_EQ(GS::RUNNING, m.state());
}

TEST(GameState, PauseFromIdleRejected)  // A3:没开始就暂停 → 拒绝,仍 IDLE
{
  GameStateMachine m;
  auto r = m.handle_pause_toggle();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(GS::IDLE, m.state());
}

TEST(GameState, AllEnemiesDeadWins)  // A4:RUNNING 下敌全灭 → WIN
{
  GameStateMachine m;
  m.handle_start();
  m.on_enemy_alive_count(1);
  EXPECT_EQ(GS::RUNNING, m.state());
  m.on_enemy_alive_count(0);
  EXPECT_EQ(GS::WIN, m.state());
}

TEST(GameState, PlayerDeadLoses)  // A5:RUNNING 下玩家死 → LOSE
{
  GameStateMachine m;
  m.handle_start();
  m.on_player_alive(false);
  EXPECT_EQ(GS::LOSE, m.state());
}

TEST(GameState, EndWhilePausedTakesEffect)  // A6:PAUSED 下胜负 → 生效
{
  GameStateMachine m;
  m.handle_start();
  m.handle_pause_toggle();
  EXPECT_EQ(GS::PAUSED, m.state());
  m.on_enemy_alive_count(0);  // 暂停前的最后一炮结算
  EXPECT_EQ(GS::WIN, m.state());
}

TEST(GameState, NoRestartAfterEnd)  // A7:终局后 start 被拒;pause 也无效
{
  GameStateMachine m;
  m.handle_start();
  m.on_player_alive(false);
  EXPECT_EQ(GS::LOSE, m.state());
  auto r = m.handle_start();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(GS::LOSE, m.state());
  EXPECT_FALSE(m.handle_pause_toggle().ok);
}

TEST(GameState, NoEndBeforeStart)  // 边界:IDLE 下死信号不应误触发终局
{
  GameStateMachine m;
  m.on_enemy_alive_count(0);
  m.on_player_alive(false);
  EXPECT_EQ(GS::IDLE, m.state());
  EXPECT_TRUE(m.handle_start().ok);  // 仍可正常开始
  EXPECT_EQ(GS::RUNNING, m.state());
}

TEST(GameState, DuplicateStartRejected)  // 幂等:RUNNING 下再 start 拒绝
{
  GameStateMachine m;
  m.handle_start();
  EXPECT_FALSE(m.handle_start().ok);
  EXPECT_EQ(GS::RUNNING, m.state());
}
