// powerup_manager_node:道具系统——随机刷新、玩家拾取、效果广播
//
// 刷新:spawn_period(默认 10s)定时,场上道具 < max_on_field(默认 5)时
//   经 /spawn_entity 生成一个静态球体(powerup.sdf,无插件,不受 B6
//   发现的多插件串扰 bug 影响);位置 ±12 随机,避开 6 个掩体(重试 10 次)
// 拾取:/model_states 里玩家与任一道具距离 < pick_radius(1.0m)→
//   /delete_entity 删除道具 + 广播 /powerup_collected(PowerupEvent,
//   collector 恒为 player——需求卡:道具只强化玩家)
// 效果生效在各消费者:keyboard(speed_up)、player_tank_node(rapid_fire)、
//   combat_system_node(damage_up/repair)各自订阅同一事件,互不耦合
// 冻结:非 RUNNING 不刷新、不拾取(与全场一致)
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tank_msgs/msg/game_state.hpp"
#include "tank_msgs/msg/powerup_event.hpp"
#include "tank_nodes/powerup_effects.hpp"

class PowerupManagerNode : public rclcpp::Node
{
public:
  explicit PowerupManagerNode(const rclcpp::NodeOptions & options)
  : Node("powerup_manager_node", options)
  {
    spawn_period_ = declare_parameter<double>("spawn_period", 10.0);
    max_on_field_ = declare_parameter<int>(
      "max_on_field", tank_nodes::PowerupEffects::kMaxOnField);
    pick_radius_ = declare_parameter<double>("pick_radius", 1.0);

    std::string sdf_path = ament_index_cpp::get_package_share_directory(
      "tank_description") + "/sdf/powerup.sdf";
    std::ifstream f(sdf_path);
    std::stringstream ss;
    ss << f.rdbuf();
    powerup_sdf_ = ss.str();
    if (powerup_sdf_.find("<model") == std::string::npos) {
      RCLCPP_FATAL(get_logger(), "道具 SDF 读取失败:%s", sdf_path.c_str());
      throw std::runtime_error("powerup.sdf missing");
    }

    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states", 10,
      [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        on_model_states(msg);
      });
    game_state_sub_ = create_subscription<tank_msgs::msg::GameState>(
      "/game_state", 10,
      [this](const tank_msgs::msg::GameState::SharedPtr msg) {
        game_state_ = msg->state;
      });
    event_pub_ = create_publisher<tank_msgs::msg::PowerupEvent>(
      "/powerup_collected", 10);
    spawn_cli_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    delete_cli_ = create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

    spawn_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() {try_spawn();});

    RCLCPP_INFO(get_logger(),
      "道具系统就绪:每 %.0fs 刷新,场上最多 %d 个,拾取半径 %.1fm",
      spawn_period_, max_on_field_, pick_radius_);
  }

private:
  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (game_state_ != tank_msgs::msg::GameState::RUNNING) {return;}
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] != "player") {continue;}
      const double px = msg->pose[i].position.x;
      const double py = msg->pose[i].position.y;
      // 遍历场上道具,距离内即拾取(登记即时摘除,防重复拾取)
      for (auto it = on_field_.begin(); it != on_field_.end(); ) {
        const double d = std::hypot(it->second.x - px, it->second.y - py);
        if (d < pick_radius_) {
          collect(it->first, it->second.type);
          it = on_field_.erase(it);
        } else {
          ++it;
        }
      }
      return;  // 只关心玩家
    }
  }

  void collect(const std::string & name, const std::string & type)
  {
    tank_msgs::msg::PowerupEvent ev;
    ev.powerup_type = type;
    ev.collector = "player";
    event_pub_->publish(ev);
    if (delete_cli_->service_is_ready()) {
      auto req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
      req->name = name;
      delete_cli_->async_send_request(req);
    }
    RCLCPP_INFO(get_logger(), "玩家拾取道具:%s(%s)", type.c_str(), name.c_str());
  }

  void try_spawn()
  {
    if (game_state_ != tank_msgs::msg::GameState::RUNNING) {return;}
    if (static_cast<int>(on_field_.size()) >= max_on_field_) {return;}
    if (last_spawn_sim_ > 0.0 &&
      this->now().seconds() - last_spawn_sim_ < spawn_period_)
    {
      return;
    }
    last_spawn_sim_ = this->now().seconds();
    if (!spawn_cli_->service_is_ready()) {return;}

    // 位置:±12 随机、避开掩体(重试 10 次;掩体是硬编码的 6 个箱子)
    double x = 0.0, y = 0.0;
    for (int attempt = 0; attempt < 10; ++attempt) {
      x = pos_dist_(rng_);
      y = pos_dist_(rng_);
      if (clear_of_bunkers(x, y)) {break;}
    }

    const std::string type = TYPES_[type_rng_(rng_)];
    const std::string name = "powerup_" + std::to_string(++seq_);

    auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    req->name = name;
    req->xml = powerup_sdf_;
    req->initial_pose.position.x = x;
    req->initial_pose.position.y = y;
    req->initial_pose.position.z = 0.0;
    req->initial_pose.orientation.w = 1.0;
    spawn_cli_->async_send_request(req,
      [this, name, type, x, y](
        rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture fut) {
        if (fut.get()->success) {
          // static 模型不会动,登记 spawn 位姿即可(不必逐拍读位置)
          on_field_[name] = Item{x, y, type};
          RCLCPP_INFO(get_logger(), "刷新道具 %s(%s) 于 (%.1f, %.1f)",
            name.c_str(), type.c_str(), x, y);
        } else {
          RCLCPP_WARN(get_logger(), "道具 %s 生成失败:%s",
            name.c_str(), fut.get()->status_message.c_str());
        }
      });
  }

  static bool clear_of_bunkers(double x, double y)
  {
    static const std::vector<std::pair<double, double>> kBunkers = {
      {5, 5}, {-5, -5}, {6, -6}, {-6, 6}, {0, 8}, {0, -8}};
    for (const auto & b : kBunkers) {
      if (std::hypot(b.first - x, b.second - y) < 2.5) {return false;}
    }
    return true;
  }

  double spawn_period_ = 10.0;
  int max_on_field_ = 5;
  double pick_radius_ = 1.0;
  double last_spawn_sim_ = 0.0;
  uint64_t seq_ = 0;
  uint8_t game_state_ = tank_msgs::msg::GameState::IDLE;

  std::string powerup_sdf_;
  // 场上道具登记(spawn 位姿 + 类型;static 模型不动)
  struct Item
  {
    double x = 0.0;
    double y = 0.0;
    std::string type;
  };
  std::map<std::string, Item> on_field_;
  const std::vector<std::string> TYPES_ = {
    "rapid_fire", "damage_up", "speed_up", "repair"};
  std::mt19937 rng_{std::random_device{}()};
  std::uniform_real_distribution<double> pos_dist_{-12.0, 12.0};
  std::uniform_int_distribution<size_t> type_rng_{0, 3};

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<tank_msgs::msg::GameState>::SharedPtr game_state_sub_;
  rclcpp::Publisher<tank_msgs::msg::PowerupEvent>::SharedPtr event_pub_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_cli_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_cli_;
  rclcpp::TimerBase::SharedPtr spawn_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PowerupManagerNode>(rclcpp::NodeOptions()));
  return 0;
}
