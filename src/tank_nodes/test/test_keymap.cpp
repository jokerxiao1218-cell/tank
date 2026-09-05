// E 组测试:键盘解析纯逻辑(KeymapParser)
// 用例对应设计文档第 4 节 E 组(teleop 语义改版,变更已记录)
#include <gtest/gtest.h>

#include "tank_nodes/keymap_parser.hpp"
#include "tank_msgs/msg/game_state.hpp"

using tank_nodes::KeymapParser;
using GS = tank_msgs::msg::GameState;

TEST(Keymap, ForwardAndStop)  // E1:按 w 前进 1.0;按 x 停
{
  KeymapParser p;
  p.on_key('w');
  EXPECT_DOUBLE_EQ(1.0, p.linear());
  EXPECT_DOUBLE_EQ(0.0, p.angular());
  p.on_key('x');
  EXPECT_DOUBLE_EQ(0.0, p.linear());
}

TEST(Keymap, ReverseAndTurn)  // E2:s 后退 -1.0;a 左转 +1.0、d 右转 -1.0;组合复合生效
{
  KeymapParser p;
  p.on_key('s');
  EXPECT_DOUBLE_EQ(-1.0, p.linear());
  p.on_key('a');
  EXPECT_DOUBLE_EQ(-1.0, p.linear());   // 前后与转向互不干扰
  EXPECT_DOUBLE_EQ(1.0, p.angular());
  KeymapParser q;
  q.on_key('d');
  EXPECT_DOUBLE_EQ(-1.0, q.angular());
}

TEST(Keymap, IdleLockout)  // E3:IDLE 下运动/开炮全部锁死,Enter/P 放行
{
  KeymapParser p(GS::IDLE);
  p.on_key('w');
  EXPECT_DOUBLE_EQ(0.0, p.linear());
  p.on_key(' ');
  EXPECT_FALSE(p.pop_fire_requested());
  p.on_key('x');
  p.on_key('\n');
  EXPECT_TRUE(p.pop_start_requested());  // IDLE 按 Enter 请求开始(合法)
  EXPECT_FALSE(p.pop_fire_requested());  // 请求不会重复吐出(pop 语义)
}

TEST(Keymap, StartRequestIsFlagNotQueue)  // E4:开始请求是"标志"不是"队列"
// 语义(2026-09-05 修正):连按 Enter 在取走前合并为一个请求——
// 开始游戏本来就是幂等动作,排队反而会让 game_master 收到冗余请求
{
  KeymapParser p(GS::RUNNING);
  p.on_key('\n');
  p.on_key('\n');
  EXPECT_TRUE(p.pop_start_requested());    // 两次 Enter 合并成一个请求
  EXPECT_FALSE(p.pop_start_requested());  // 取走即清
  p.on_key('\n');                          // 再按再产生
  EXPECT_TRUE(p.pop_start_requested());
}

TEST(Keymap, UnknownKeyIgnored)  // E5:未知键不改变任何状态、不崩
{
  KeymapParser p;
  p.on_key('w');
  p.on_key('z');   // 未定义键
  p.on_key('\x1b');  // 转义序列残渣
  EXPECT_DOUBLE_EQ(1.0, p.linear());
  EXPECT_FALSE(p.pop_start_requested());
  EXPECT_FALSE(p.pop_fire_requested());
}

TEST(Keymap, TurretStep)  // E6:Q/E 炮塔步进 0.2rad,方向 Q 左(+)E 右(-)
{
  KeymapParser p;
  p.on_key('q');
  EXPECT_DOUBLE_EQ(0.2, p.turret_target());
  p.on_key('q');
  p.on_key('q');
  EXPECT_DOUBLE_EQ(0.6, p.turret_target());
  p.on_key('e');
  EXPECT_DOUBLE_EQ(0.4, p.turret_target());
}

TEST(Keymap, PausedLockoutAndPauseKey)  // E7:PAUSED 下运动锁死但 P 放行
// 语义(2026-09-05 修正):恢复后延续暂停前的运动(需求卡:"从冻结处继续,不是重开")
{
  KeymapParser p(GS::RUNNING);
  p.on_key('w');
  p.set_game_state(GS::PAUSED);
  EXPECT_DOUBLE_EQ(0.0, p.linear());       // 暂停即冻结输出
  p.on_key('a');
  EXPECT_DOUBLE_EQ(0.0, p.angular());     // 暂停时按键忽略
  p.on_key('p');
  EXPECT_TRUE(p.pop_pause_requested());   // P 在 PAUSED 下仍是合法请求(恢复)
  p.set_game_state(GS::RUNNING);
  EXPECT_DOUBLE_EQ(1.0, p.linear());     // 恢复后延续暂停前的运动(冻结式恢复)
}

TEST(Keymap, FireOnlyWhenRunning)  // E8:空格只在 RUNNING 出事件;事件 pop 一次性
{
  KeymapParser p(GS::RUNNING);
  p.on_key(' ');
  EXPECT_TRUE(p.pop_fire_requested());
  EXPECT_FALSE(p.pop_fire_requested());
}
