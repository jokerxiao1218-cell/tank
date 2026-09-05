// player_tank_node:玩家坦克执行器(键盘指令 → 仿真实体的"翻译官")
//
// 职责:
//   1. 订阅 /<prefix>/turret_cmd(绝对目标角),把位置控制命令发给
//      ros2_control 的 JointGroupPositionController(话题 Float64MultiArray)
//   2. 订阅 /game_state:非 RUNNING 时冻结一切下发(暂停的第二层:物理层由
//      game_master 调 Gazebo pause,这里是逻辑层冻结)
//   3. batch 4 将在此节点追加开炮处理(订阅 /<prefix>/fire → 生成炮弹)
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tank_msgs/msg/game_state.hpp"

class PlayerTankNode : public rclcpp::Node
{
public:
  explicit PlayerTankNode(const rclcpp::NodeOptions & options)
  : Node("player_tank_node", options)
  {
    const std::string prefix = declare_parameter<std::string>("prefix", "player");

    turret_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/" + prefix + "/turret_cmd", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        target_angle_ = msg->data;
        target_valid_ = true;
      });

    commands_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/" + prefix + "/turret_position_controller/commands", 10);

    game_state_sub_ = create_subscription<tank_msgs::msg::GameState>(
      "/game_state", 10,
      [this](const tank_msgs::msg::GameState::SharedPtr msg) {
        game_state_ = msg->state;
        // 从暂停恢复(RUNNING)时,重发一次炮塔目标,确保控制器跟得上
        if (game_state_ == tank_msgs::msg::GameState::RUNNING) {
          target_valid_ = true;
        }
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {on_timer();});  // 50Hz

    RCLCPP_INFO(get_logger(), "玩家坦克执行器就绪(prefix=%s)", prefix.c_str());
  }

private:
  void on_timer()
  {
    // 暂停/未开始时不下发任何命令(逻辑层冻结)
    if (game_state_ != tank_msgs::msg::GameState::RUNNING || !target_valid_) {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {target_angle_};
    commands_pub_->publish(msg);
    target_valid_ = false;  // 目标已下发,等下一个变化
  }

  uint8_t game_state_ = tank_msgs::msg::GameState::RUNNING;
  double target_angle_ = 0.0;
  bool target_valid_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr turret_cmd_sub_;
  rclcpp::Subscription<tank_msgs::msg::GameState>::SharedPtr game_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commands_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlayerTankNode>(rclcpp::NodeOptions()));
  return 0;
}
