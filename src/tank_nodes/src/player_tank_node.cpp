// player_tank_node:玩家坦克执行器(键盘指令 → 仿真实体的"翻译官")
//
// 职责:
//   1. 订阅 /<prefix>/turret_cmd(绝对目标角)→ 转发给 ros2_control
//      JointGroupPositionController(Float64MultiArray)
//   2. 订阅 /<prefix>/fire → 开炮:spawn_entity 生成炮弹 + set_entity_state 设初速
//      开炮点 = 坦克位置 + 朝向(车体 yaw + 炮塔角)前伸 1.2m、高 0.75m(炮口)
//   3. 订阅 /model_states 缓存自身位姿(开炮方向计算用)
//   4. 订阅 /game_state:非 RUNNING 冻结一切(暂停第二层:逻辑冻结)
//
// 炮弹初速两步设(spawn 无初速字段):spawn → set_entity_state 带 twist,
// 两拍间隔约数十毫秒,实测若有顿挫按设计文档 3.5.4 备选方案换 apply_body_wrench
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tank_msgs/msg/game_state.hpp"
#include "tank_msgs/msg/powerup_event.hpp"
#include "tank_nodes/damage_calculator.hpp"
#include "tank_nodes/powerup_effects.hpp"

class PlayerTankNode : public rclcpp::Node
{
public:
  explicit PlayerTankNode(const rclcpp::NodeOptions & options)
  : Node("player_tank_node", options)
  {
    const std::string prefix = declare_parameter<std::string>("prefix", "player");
    prefix_ = prefix;
    fire_cooldown_ = declare_parameter<double>("fire_cooldown", 1.5);  // 需求卡:1.5s
    bullet_speed_ = declare_parameter<double>(
      "bullet_speed", tank_nodes::DamageCalculator::kBulletSpeed);

    // ---- 炮弹 SDF 读入一次,后续每发只换名字 ----
    std::string sdf_path = ament_index_cpp::get_package_share_directory(
      "tank_description") + "/sdf/bullet.sdf";
    std::ifstream f(sdf_path);
    std::stringstream ss;
    ss << f.rdbuf();
    bullet_sdf_ = ss.str();
    if (bullet_sdf_.find("<model") == std::string::npos) {
      RCLCPP_FATAL(get_logger(), "炮弹 SDF 读取失败:%s", sdf_path.c_str());
      throw std::runtime_error("bullet.sdf missing");
    }

    // ---- 话题 ----
    turret_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/" + prefix + "/turret_cmd", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        target_angle_ = msg->data;
        target_valid_ = true;
      });
    fire_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/" + prefix + "/fire", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {on_fire();});
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states", 10,
      [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {on_model_states(msg);});
    game_state_sub_ = create_subscription<tank_msgs::msg::GameState>(
      "/game_state", 10,
      [this](const tank_msgs::msg::GameState::SharedPtr msg) {
        game_state_ = msg->state;
        if (game_state_ == tank_msgs::msg::GameState::RUNNING) {
          target_valid_ = true;  // 恢复时补发一次炮塔目标
        }
      });
    commands_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/" + prefix + "/turret_position_controller/commands", 10);

    // 道具(batch 7):rapid_fire 生效——开炮冷却 ×0.5(执行器前缀匹配,
    // 玩家/敌人执行器通用,敌人不会收到 collector=player 的事件)
    powerup_sub_ = create_subscription<tank_msgs::msg::PowerupEvent>(
      "/powerup_collected", 10,
      [this](const tank_msgs::msg::PowerupEvent::SharedPtr msg) {
        if (msg->collector != prefix_ || msg->powerup_type != "rapid_fire") {return;}
        fire_cooldown_ = tank_nodes::PowerupEffects::apply_rapid_fire(fire_cooldown_);
        RCLCPP_INFO(get_logger(), "射速强化!当前冷却 %.2fs", fire_cooldown_);
      });

    // ---- Gazebo 服务客户端 ----
    spawn_cli_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    set_state_cli_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {on_timer();});  // 50Hz

    // 冷却基准用同一时钟初始化(this->now() 是系统钟;若用 ROS_TIME 的 0
    // 与系统钟相减会抛"不同时间源"异常,导致开炮回调死在冷却判断上)
    last_fire_time_ = this->now();

    RCLCPP_INFO(get_logger(), "玩家坦克执行器就绪(prefix=%s,冷却 %.1fs)",
      prefix.c_str(), fire_cooldown_);
  }

private:
  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == prefix_) {
        tank_pose_ = msg->pose[i];
        pose_valid_ = true;
        return;
      }
    }
  }

  // pose 里四元数取 yaw(偏航角)
  static double yaw_of(const geometry_msgs::msg::Pose & p)
  {
    const double siny = 2.0 * (p.orientation.w * p.orientation.z +
      p.orientation.x * p.orientation.y);
    const double cosy = 1.0 - 2.0 * (p.orientation.y * p.orientation.y +
      p.orientation.z * p.orientation.z);
    return std::atan2(siny, cosy);
  }

  void on_fire()
  {
    if (game_state_ != tank_msgs::msg::GameState::RUNNING) {return;}

    const rclcpp::Time now = this->now();
    if ((now - last_fire_time_).seconds() < fire_cooldown_) {
      RCLCPP_DEBUG(get_logger(), "开炮冷却中");
      return;
    }
    if (!pose_valid_ || !spawn_cli_->service_is_ready()) {return;}

    // 炮弹方向 = 车体 yaw + 炮塔角;出生点在炮口(前伸 1.2m,高 0.75m)
    const double dir = yaw_of(tank_pose_) + target_angle_;
    const double muzzle_x = tank_pose_.position.x + 1.2 * std::cos(dir);
    const double muzzle_y = tank_pose_.position.y + 1.2 * std::sin(dir);

    auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    // 弹名带发射者 prefix:多个敌人执行器各自计数,共用 bullet_e_ 前缀
    // 会撞名(B6 实测 "Entity already exists" 生成失败互斥)
    req->name = "bullet_" + prefix_ + "_" + std::to_string(++bullet_seq_);
    req->xml = bullet_sdf_;
    req->initial_pose.position.x = muzzle_x;
    req->initial_pose.position.y = muzzle_y;
    req->initial_pose.position.z = tank_pose_.position.z + 0.55;
    req->initial_pose.orientation.w = 1.0;

    const double vx = bullet_speed_ * std::cos(dir);
    const double vy = bullet_speed_ * std::sin(dir);
    const std::string bullet_name = req->name;
    const geometry_msgs::msg::Pose muzzle_pose = req->initial_pose;  // set 速度时位姿必须一并带上

    spawn_cli_->async_send_request(req,
      [this, bullet_name, vx, vy, muzzle_pose](
        rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture fut) {
        if (!fut.get()->success) {
          RCLCPP_WARN(get_logger(), "炮弹 %s 生成失败:%s",
            bullet_name.c_str(), fut.get()->status_message.c_str());
          return;
        }
        // 第二步:设初速。坑:SetEntityState 会整体覆盖 EntityState,
        // 若 pose 不填,炮弹会被重置到 (0,0,0)(玩家坦克体内)→ 物理爆冲消失
        auto st = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        st->state.name = bullet_name;
        st->state.pose = muzzle_pose;
        st->state.twist.linear.x = vx;
        st->state.twist.linear.y = vy;
        set_state_cli_->async_send_request(st);
      });
    last_fire_time_ = now;
    RCLCPP_DEBUG(get_logger(), "开炮:%s 速度(%.1f, %.1f)", bullet_name.c_str(), vx, vy);
  }

  void on_timer()
  {
    if (game_state_ != tank_msgs::msg::GameState::RUNNING || !target_valid_) {return;}
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {target_angle_};
    commands_pub_->publish(msg);
    target_valid_ = false;
  }

  std::string prefix_;
  uint8_t game_state_ = tank_msgs::msg::GameState::RUNNING;
  double fire_cooldown_ = 1.5;
  double bullet_speed_ = 30.0;
  double target_angle_ = 0.0;
  bool target_valid_ = false;
  geometry_msgs::msg::Pose tank_pose_;
  bool pose_valid_ = false;
  uint64_t bullet_seq_ = 0;
  rclcpp::Time last_fire_time_;  // 构造函数里与 this->now() 同源初始化
  std::string bullet_sdf_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr turret_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fire_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<tank_msgs::msg::GameState>::SharedPtr game_state_sub_;
  rclcpp::Subscription<tank_msgs::msg::PowerupEvent>::SharedPtr powerup_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commands_pub_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_cli_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlayerTankNode>(rclcpp::NodeOptions()));
  return 0;
}
