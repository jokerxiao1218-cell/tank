// hud_node:终端 HUD——一秒一行"战况直播"
//
// 订阅:/game_state(状态+剩余敌数)、/tank_status(玩家血量)、
//       /powerup_collected(强化记录)
// 输出:1Hz 一行式 HUD(带 ANSI 颜色;不清屏,日志文件/tail -f 都友好):
//   [HUD] 运行中|敌3|玩家 75/100 [███████░░░]|强化: 速度1 火力2
//
// 血量颜色:绿>50 黄>25 红≤25;状态:运行绿/暂停黄/胜绿/败红
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tank_msgs/msg/game_state.hpp"
#include "tank_msgs/msg/powerup_event.hpp"
#include "tank_msgs/msg/tank_status.hpp"

namespace
{
const char * kReset = "\033[0m";
const char * kGreen = "\033[32m";
const char * kYellow = "\033[33m";
const char * kRed = "\033[31m";
const char * kCyan = "\033[36m";
}  // namespace

class HudNode : public rclcpp::Node
{
public:
  explicit HudNode(const rclcpp::NodeOptions & options)
  : Node("hud_node", options)
  {
    state_sub_ = create_subscription<tank_msgs::msg::GameState>(
      "/game_state", 10,
      [this](const tank_msgs::msg::GameState::SharedPtr msg) {
        state_ = msg->state;
        // 敌人数不取 msg->enemy_alive:开局阶段总控还没统计到,显示 0 会误导
      });
    status_sub_ = create_subscription<tank_msgs::msg::TankStatus>(
      "/tank_status", 10,
      [this](const tank_msgs::msg::TankStatus::SharedPtr msg) {
        // 敌人数量自己数(战斗系统是血量/存活的事实源;开局阶段
        // game_master 的 enemy_alive 广播还没统计到,直接显示会误导)
        if (msg->is_player) {
          health_ = msg->health;
          alive_ = msg->alive;
        } else {
          auto it = enemy_alive_map_.find(msg->tank_name);
          if (it == enemy_alive_map_.end()) {
            enemy_alive_map_[msg->tank_name] = msg->alive;
          } else {
            it->second = msg->alive;
          }
        }
      });
    powerup_sub_ = create_subscription<tank_msgs::msg::PowerupEvent>(
      "/powerup_collected", 10,
      [this](const tank_msgs::msg::PowerupEvent::SharedPtr msg) {
        // 最近 3 条强化记录(HUD 显示用)
        if (history_.size() >= 3) {history_.pop_front();}
        history_.push_back(label_of(msg->powerup_type));
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000), [this]() {render();});

    RCLCPP_INFO(get_logger(), "HUD 就绪");
  }

private:
  static std::string label_of(const std::string & type)
  {
    if (type == "rapid_fire") {return "射速";}
    if (type == "damage_up") {return "火力";}
    if (type == "speed_up") {return "速度";}
    if (type == "repair") {return "维修";}
    return type;
  }

  void render()
  {
    const char * sc;
    std::string state_str;
    switch (state_) {
      case tank_msgs::msg::GameState::RUNNING: sc = kGreen; state_str = "运行中"; break;
      case tank_msgs::msg::GameState::PAUSED: sc = kYellow; state_str = "已暂停"; break;
      case tank_msgs::msg::GameState::WIN: sc = kGreen; state_str = "胜利!敌人全灭"; break;
      case tank_msgs::msg::GameState::LOSE: sc = kRed; state_str = "失败,按重启"; break;
      default: sc = kCyan; state_str = "准备中(自动开始)"; break;
    }

    int enemies = 0;
    for (const auto & [name, a] : enemy_alive_map_) {
      if (a) {++enemies;}
    }

    // 10 格血条
    const int bars = alive_ ? static_cast<int>(health_ / 10.0 + 0.5) : 0;
    const char * hc = alive_ ? (health_ > 50 ? kGreen : (health_ > 25 ? kYellow : kRed)) : kRed;
    std::string bar(10, '-');
    for (int i = 0; i < bars && i < 10; ++i) {bar[i] = '#';}

    std::string line = std::string("[HUD] ") + sc + state_str + kReset +
      " | 敌" + std::to_string(enemies) + " | " + hc +
      "玩家 " + std::to_string(static_cast<int>(health_)) + "/100 [" + bar + "]" + kReset;
    if (!history_.empty()) {
      line += " | 强化: ";
      bool first = true;
      for (const auto & h : history_) {
        if (!first) {line += " ";}
        line += h;
        first = false;
      }
    }
    if (!alive_) {line += " (被击毁)";}
    printf("%s\n", line.c_str());
    fflush(stdout);
  }

  uint8_t state_ = tank_msgs::msg::GameState::IDLE;
  double health_ = 100.0;
  bool alive_ = true;
  std::map<std::string, bool> enemy_alive_map_;
  std::deque<std::string> history_;

  rclcpp::Subscription<tank_msgs::msg::GameState>::SharedPtr state_sub_;
  rclcpp::Subscription<tank_msgs::msg::TankStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<tank_msgs::msg::PowerupEvent>::SharedPtr powerup_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HudNode>(rclcpp::NodeOptions()));
  return 0;
}
