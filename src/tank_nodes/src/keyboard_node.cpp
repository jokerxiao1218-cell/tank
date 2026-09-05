// keyboard_node:终端键盘 → 坦克指令(坦克大战的"手柄")
//
// 职责:
//   1. 独立线程以 termios raw 模式读终端按键(不回显、不用回车),喂给 KeymapParser
//   2. 10Hz 定时器把解析结果发成 ROS2 话题:
//      /<prefix>/cmd_vel   geometry_msgs/Twist        底盘速度(Gazebo planar_move 直接消费)
//      /<prefix>/turret_cmd std_msgs/Float64          炮塔绝对目标角
//      /<prefix>/fire      std_msgs/Empty             开炮事件(batch 4 的 player_tank 消费)
//   3. Enter/P 的开始/暂停请求:batch 5 接 game_master 服务,当前先记日志
//
// 退出礼仪:恢复终端属性(raw 模式不关,用户终端会"失灵")——
// 用 RAII + SIGINT 双保险,进程结束前 tcsetattr 复原
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tank_msgs/msg/game_state.hpp"
#include "tank_msgs/msg/powerup_event.hpp"
#include "tank_nodes/keymap_parser.hpp"
#include "tank_nodes/powerup_effects.hpp"

namespace
{
termios g_saved_tio;
std::atomic<bool> g_terminal_raw{false};

void restore_terminal()
{
  if (g_terminal_raw.exchange(false)) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_tio);
  }
}

void handle_sigint(int)
{
  restore_terminal();
  rclcpp::shutdown();
}
}  // namespace

class KeyboardNode : public rclcpp::Node
{
public:
  explicit KeyboardNode(const rclcpp::NodeOptions & options)
  : Node("keyboard_node", options)
  {
    // prefix 参数:玩家/敌人共用节点代码,batch 6 敌人不用键盘,但保持参数化一致性
    const std::string prefix = declare_parameter<std::string>("prefix", "player");
    prefix_ = prefix;
    // speed_scale:速度道具(batch 7)通过参数/话题注入,当前默认 1.0
    speed_scale_ = declare_parameter<double>("speed_scale", 1.0);

    parser_ = std::make_unique<tank_nodes::KeymapParser>();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/" + prefix + "/cmd_vel", 10);
    turret_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/" + prefix + "/turret_cmd", 10);
    fire_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/" + prefix + "/fire", 10);

    // 全局游戏状态广播(game_master 发布,batch 5 上线;收不到消息前保持初始 RUNNING)
    auto game_state_cb =
      [this](const tank_msgs::msg::GameState::SharedPtr msg) {
        parser_->set_game_state(msg->state);
      };
    game_state_sub_ = create_subscription<tank_msgs::msg::GameState>(
      "/game_state", 10, game_state_cb);

    // 道具(batch 7):speed_up 生效——底盘速度倍率 +0.3 封顶 1.9
    powerup_sub_ = create_subscription<tank_msgs::msg::PowerupEvent>(
      "/powerup_collected", 10,
      [this](const tank_msgs::msg::PowerupEvent::SharedPtr msg) {
        if (msg->collector != prefix_ || msg->powerup_type != "speed_up") {return;}
        speed_scale_ = tank_nodes::PowerupEffects::apply_speed_up(speed_scale_);
        RCLCPP_INFO(get_logger(), "速度强化!当前倍率 %.1f", speed_scale_);
      });

    // 开始/暂停服务客户端(game_master 提供)
    start_cli_ = create_client<std_srvs::srv::Trigger>("/game/start");
    pause_cli_ = create_client<std_srvs::srv::Trigger>("/game/pause");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {on_timer();});  // 10Hz

    start_keyboard_thread();

    RCLCPP_INFO(get_logger(),
      "键盘就绪(需要焦点在这个终端):W/S 前进后退 A/D 转向 X 急停 "
      "Q/E 炮塔 空格 开炮 Enter 开始 P 暂停");
  }

  ~KeyboardNode() override {restore_terminal();}

private:
  void start_keyboard_thread()
  {
    termios raw;
    tcgetattr(STDIN_FILENO, &g_saved_tio);
    raw = g_saved_tio;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    g_terminal_raw = true;
    std::signal(SIGINT, handle_sigint);
    atexit(restore_terminal);

    key_thread_ = std::thread([this]() {
      char c;
      while (rclcpp::ok() && running_) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
          std::lock_guard<std::mutex> lock(parser_mutex_);
          parser_->on_key(c);
        }
      }
    });
  }

  void on_timer()
  {
    double linear, angular, turret_target;
    bool start_req, pause_req, fire_req;
    {
      std::lock_guard<std::mutex> lock(parser_mutex_);
      linear = parser_->linear() * speed_scale_;
      angular = parser_->angular();
      turret_target = parser_->turret_target();
      start_req = parser_->pop_start_requested();
      pause_req = parser_->pop_pause_requested();
      fire_req = parser_->pop_fire_requested();
      const uint8_t state = parser_->game_state();
      if (state != last_state_) {
        last_state_ = state;
      }
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    cmd_vel_pub_->publish(twist);

    // 炮塔目标只在变化时发(减少无谓流量;player_tank_node 缓存目标)
    if (turret_target != last_turret_target_) {
      std_msgs::msg::Float64 msg;
      msg.data = turret_target;
      turret_pub_->publish(msg);
      last_turret_target_ = turret_target;
    }

    if (fire_req) {
      fire_pub_->publish(std_msgs::msg::Empty());
    }
    if (start_req && start_cli_->service_is_ready()) {
      start_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
          if (!fut.get()->success) {
            RCLCPP_INFO(get_logger(), "开始被拒:%s", fut.get()->message.c_str());
          }
        });
    }
    if (pause_req && pause_cli_->service_is_ready()) {
      pause_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
          if (fut.get()->success) {
            RCLCPP_INFO(get_logger(), "已请求暂停/恢复");
          }
        });
    }
  }

  std::unique_ptr<tank_nodes::KeymapParser> parser_;
  std::mutex parser_mutex_;
  std::thread key_thread_;
  std::atomic<bool> running_{true};
  double speed_scale_ = 1.0;
  double last_turret_target_ = 0.0;
  uint8_t last_state_ = 0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turret_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fire_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_cli_;
  rclcpp::Subscription<tank_msgs::msg::GameState>::SharedPtr game_state_sub_;
  rclcpp::Subscription<tank_msgs::msg::PowerupEvent>::SharedPtr powerup_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string prefix_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardNode>(rclcpp::NodeOptions()));
  restore_terminal();
  return 0;
}
