// game_master_node:游戏总控——状态机执行者 + 全场状态广播 + 两层暂停的"物理层"
//
// 服务(键盘等调用):/game/start、/game/pause(Trigger)
// 广播(所有人订阅):/game_state 10Hz(各节点靠它冻结/解冻,即暂停的"逻辑层")
// 输入:/tank_status(战斗系统的血量/存活)
// 物理层暂停:切 PAUSED 时调 Gazebo /pause_physics(子弹悬停),恢复时 /unpause_physics
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tank_msgs/msg/game_state.hpp"
#include "tank_msgs/msg/tank_status.hpp"
#include "tank_nodes/game_state_machine.hpp"

class GameMasterNode : public rclcpp::Node
{
public:
  explicit GameMasterNode(const rclcpp::NodeOptions & options)
  : Node("game_master_node", options)
  {
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/game/start",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        const auto r = sm_.handle_start();
        resp->success = r.ok;
        resp->message = r.message;
        if (r.ok) {RCLCPP_INFO(get_logger(), "游戏开始!");}
      });
    pause_srv_ = create_service<std_srvs::srv::Trigger>(
      "/game/pause",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        const auto r = sm_.handle_pause_toggle();
        resp->success = r.ok;
        resp->message = r.message;
        if (!r.ok) {return;}
        if (sm_.state() == tank_msgs::msg::GameState::PAUSED) {
          call_empty(pause_cli_, "/pause_physics");   // 物理层:子弹悬停
          RCLCPP_INFO(get_logger(), "已暂停(物理+逻辑两层冻结)");
        } else {
          call_empty(unpause_cli_, "/unpause_physics");
          RCLCPP_INFO(get_logger(), "已恢复");
        }
      });

    status_sub_ = create_subscription<tank_msgs::msg::TankStatus>(
      "/tank_status", 10,
      [this](const tank_msgs::msg::TankStatus::SharedPtr msg) {
        on_tank_status(msg);
      });

    state_pub_ = create_publisher<tank_msgs::msg::GameState>("/game_state", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {
        tank_msgs::msg::GameState msg;
        msg.state = sm_.state();
        msg.enemy_alive = static_cast<uint8_t>(sm_.enemy_alive());
        state_pub_->publish(msg);
      });

    pause_cli_ = create_client<std_srvs::srv::Empty>("/pause_physics");
    unpause_cli_ = create_client<std_srvs::srv::Empty>("/unpause_physics");

    RCLCPP_INFO(get_logger(), "游戏总控就绪:按 Enter 开始,P 暂停/恢复");
  }

private:
  void on_tank_status(const tank_msgs::msg::TankStatus::SharedPtr msg)
  {
    latest_[msg->tank_name] = *msg;
    if (!msg->is_player) {seen_any_enemy_ = true;}

    // 敌全灭判定:只在见过敌人状态后才启用(防止启动初期误判 A 组边界)
    if (seen_any_enemy_) {
      int alive = 0;
      for (const auto & [name, s] : latest_) {
        if (!s.is_player && s.alive) {++alive;}
      }
      sm_.on_enemy_alive_count(alive);
    }
    const auto it = latest_.find("player");
    if (it != latest_.end()) {
      sm_.on_player_alive(it->second.alive);
    }

    // 胜负日志只在状态变化时打一次(status 20Hz 会连刷)
    if (sm_.state() != last_state_) {
      last_state_ = sm_.state();
      if (sm_.state() == tank_msgs::msg::GameState::WIN) {
        RCLCPP_INFO(get_logger(), "===胜利!敌人全灭===");
      } else if (sm_.state() == tank_msgs::msg::GameState::LOSE) {
        RCLCPP_INFO(get_logger(), "===失败!玩家被击毁===");
      }
    }
  }

  void call_empty(
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli,
    const std::string & name)
  {
    if (!cli->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s 服务不可用", name.c_str());
      return;
    }
    cli->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  }

  tank_nodes::GameStateMachine sm_;
  std::map<std::string, tank_msgs::msg::TankStatus> latest_;
  bool seen_any_enemy_ = false;
  uint8_t last_state_ = tank_msgs::msg::GameState::IDLE;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Subscription<tank_msgs::msg::TankStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<tank_msgs::msg::GameState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_cli_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr unpause_cli_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameMasterNode>(rclcpp::NodeOptions()));
  return 0;
}
