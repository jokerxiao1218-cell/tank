// enemy_ai_node:敌人 AI 节点(每辆敌人坦克一个实例,prefix 参数区分)
//
// 职责:
//   1. 订阅 /model_states:缓存自身 + 玩家位姿
//   2. 订阅 /game_state:非 RUNNING 时决策层自然输出全零(冻结)
//   3. 10Hz 调 EnemyDecision::decide(纯逻辑,单测 D 组覆盖)→ 发布:
//      /<prefix>/cmd_vel(底盘,planar_move 吃)
//      /<prefix>/turret_cmd(炮塔关节角,player_tank_node 执行器吃)
//      /<prefix>/fire(开炮,player_tank_node 执行器 spawn 炮弹)
//   4. 巡逻目标点:随机采样 ±12m(30×30 场地内安全区),
//      到达或 20s 超时(防撞墙卡死)换新目标
//
// use_sim_time:=true:now() 走仿真钟,暂停时冷却也冻结(与物理一致)
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tank_msgs/msg/game_state.hpp"
#include "tank_nodes/enemy_decision.hpp"

class EnemyAiNode : public rclcpp::Node
{
public:
  explicit EnemyAiNode(const rclcpp::NodeOptions & options)
  : Node("enemy_ai_node", options)
  {
    prefix_ = declare_parameter<std::string>("prefix", "enemy_1");
    patrol_timeout_ = declare_parameter<double>("patrol_timeout", 20.0);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/" + prefix_ + "/cmd_vel", 10);
    turret_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/" + prefix_ + "/turret_cmd", 10);
    fire_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/" + prefix_ + "/fire", 10);

    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states", 10,
      [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == prefix_) {
            own_pose_ = msg->pose[i];
            own_valid_ = true;
          } else if (msg->name[i] == "player") {
            player_pose_ = msg->pose[i];
            player_valid_ = true;
          }
        }
      });
    game_state_sub_ = create_subscription<tank_msgs::msg::GameState>(
      "/game_state", 10,
      [this](const tank_msgs::msg::GameState::SharedPtr msg) {
        game_state_ = msg->state;
      });

    new_patrol_target();  // 初始巡逻点
    last_fire_ = -1000.0;           // 从未开炮(冷却立即可开)
    patrol_since_ = this->now().seconds();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {on_timer();});  // 10Hz

    RCLCPP_INFO(get_logger(), "敌人 AI 就绪(%s)", prefix_.c_str());
  }

private:
  static double yaw_of(const geometry_msgs::msg::Pose & p)
  {
    const double siny = 2.0 * (p.orientation.w * p.orientation.z +
      p.orientation.x * p.orientation.y);
    const double cosy = 1.0 - 2.0 * (p.orientation.y * p.orientation.y +
      p.orientation.z * p.orientation.z);
    return std::atan2(siny, cosy);
  }

  void new_patrol_target()
  {
    std::uniform_real_distribution<double> d(-12.0, 12.0);
    patrol_tx_ = d(rng_);
    patrol_ty_ = d(rng_);
    patrol_since_ = this->now().seconds();
  }

  void on_timer()
  {
    if (!own_valid_ || !player_valid_) {return;}

    const double now = this->now().seconds();
    const auto o = tank_nodes::EnemyDecision::decide(
      game_state_,
      own_pose_.position.x, own_pose_.position.y, yaw_of(own_pose_),
      player_pose_.position.x, player_pose_.position.y,
      patrol_tx_, patrol_ty_,
      now, last_fire_);

    // 到达巡逻点 / 超时 → 换目标(超时兜底:不避障可能卡在掩体上)
    if (o.arrived_patrol || now - patrol_since_ > patrol_timeout_) {
      new_patrol_target();
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = o.linear;
    cmd.angular.z = o.angular;
    cmd_vel_pub_->publish(cmd);

    std_msgs::msg::Float64 turret;
    turret.data = o.turret_joint;
    turret_pub_->publish(turret);

    if (o.fire) {
      fire_pub_->publish(std_msgs::msg::Empty());
      last_fire_ = now;  // 冷却计时由本节点记录,纯逻辑保持无状态
      RCLCPP_INFO(get_logger(), "%s 开炮!", prefix_.c_str());
    }
  }

  std::string prefix_;
  uint8_t game_state_ = tank_msgs::msg::GameState::IDLE;
  double patrol_timeout_ = 20.0;
  double patrol_tx_ = 0.0;
  double patrol_ty_ = 0.0;
  double patrol_since_ = 0.0;
  double last_fire_ = -1000.0;
  geometry_msgs::msg::Pose own_pose_;
  geometry_msgs::msg::Pose player_pose_;
  bool own_valid_ = false;
  bool player_valid_ = false;
  std::mt19937 rng_{std::random_device{}()};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turret_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fire_pub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<tank_msgs::msg::GameState>::SharedPtr game_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnemyAiNode>(rclcpp::NodeOptions()));
  return 0;
}
