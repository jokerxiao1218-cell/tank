// combat_system_node:战斗系统——炮弹命中判定、伤害结算、击毁、超时销毁的"裁判"
//
// 数据源:/model_states(全部实体位姿,自节流 20Hz 判定)+ /clock(仿真时间,
// 暂停时炮弹寿命不走)+ /game_state(非 RUNNING 全冻结)
//
// 实体自动发现(解耦:开炮方不需要通知谁):
//   player / enemy_* 前缀 → 坦克注册(血量 100)
//   bullet_p_* / bullet_e_* 前缀 → 炮弹注册(记出生时刻、逐拍轨迹)
// 命中:炮弹两判定拍连线与坦克中心距离 < 0.5m(线段扫过,30m/s 弹速不漏);
//       同阵营免疫(DamageCalculator::hit 内含)
// 结算:玩家/敌人炮弹均 25 伤(batch 7 道具强化在此扩展);
//       血 ≤ 0 → delete_entity 销毁坦克、置 alive=false(状态保留供 HUD)
//
// 炮弹生命周期(坑:删除是异步的,删除生效前 model_states 仍会看到该弹;
// 若命中后立即摘除登记,该弹会被误当"新弹"重注册、二次命中):
//   命中/超时 → 只标记 delete_requested 并请求删除,不再参与判定;
//   登记 convergence:每拍从名单里消失的弹才真正摘除
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tank_msgs/msg/game_state.hpp"
#include "tank_msgs/msg/tank_status.hpp"
#include "tank_nodes/damage_calculator.hpp"

class CombatSystemNode : public rclcpp::Node
{
public:
  explicit CombatSystemNode(const rclcpp::NodeOptions & options)
  : Node("combat_system_node", options)
  {
    default_health_ = declare_parameter<double>("default_health", 100.0);
    hit_radius_ = declare_parameter<double>(
      "hit_radius", tank_nodes::DamageCalculator::kHitRadius);
    bullet_lifetime_ = declare_parameter<double>(
      "bullet_lifetime", tank_nodes::DamageCalculator::kBulletLifetime);
    judge_period_ = declare_parameter<double>("judge_period", 0.05);  // 20Hz

    // 仿真时钟:暂停时炮弹寿命冻结(需求卡:暂停=冻结一切,不只是画面)
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

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
    status_pub_ = create_publisher<tank_msgs::msg::TankStatus>(
      "/tank_status", 10);
    delete_cli_ = create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

    RCLCPP_INFO(get_logger(),
      "战斗系统就绪(命中半径 %.2fm,炮弹寿命 %.1fs)", hit_radius_, bullet_lifetime_);
  }

private:
  struct TankEntry
  {
    bool is_player = false;
    double health = 100.0;
    bool alive = true;
    double x = 0.0, y = 0.0;
    bool pos_valid = false;
  };
  struct BulletEntry
  {
    double born_sim = 0.0;                 // 出生时刻(仿真秒)
    double judge_x = 0.0, judge_y = 0.0;   // 上一判定拍位置
    double cur_x = 0.0, cur_y = 0.0;       // 最新位置
    bool has_judged_prev = false;
    bool delete_requested = false;         // 命中/超时后置位,等名单消失再摘
  };

  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (game_state_ != tank_msgs::msg::GameState::RUNNING) {return;}
    const double sim = this->now().seconds();
    const bool time_to_judge = (sim - last_judge_sim_) >= judge_period_;

    // ---- 1. 名单收集 + 注册/位置更新 ----
    std::set<std::string> seen;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string & name = msg->name[i];
      const auto & pos = msg->pose[i].position;
      seen.insert(name);

      if (is_tank_name(name)) {
        auto it = tanks_.find(name);
        if (it == tanks_.end()) {
          TankEntry t;
          t.is_player = (name == "player");
          t.health = default_health_;
          tanks_[name] = t;
          RCLCPP_INFO(get_logger(), "坦克注册:%s(血量 %.0f)", name.c_str(), t.health);
          it = tanks_.find(name);
        }
        it->second.x = pos.x;
        it->second.y = pos.y;
        it->second.pos_valid = true;
      } else if (name.rfind("bullet_", 0) == 0) {
        auto it = bullets_.find(name);
        if (it == bullets_.end()) {
          BulletEntry b;  // 新弹(删除延迟窗口里的老弹因未摘除不会走到这)
          b.born_sim = sim;
          b.cur_x = pos.x;
          b.cur_y = pos.y;
          bullets_[name] = b;
        } else {
          it->second.cur_x = pos.x;
          it->second.cur_y = pos.y;
        }
      }
    }

    // ---- 2. 登记 convergence:名单里消失的弹真正摘除(坦克状态保留供 HUD) ----
    for (auto it = bullets_.begin(); it != bullets_.end(); ) {
      if (seen.count(it->first) == 0) {
        it = bullets_.erase(it);
      } else {
        ++it;
      }
    }

    if (!time_to_judge) {return;}
    last_judge_sim_ = sim;

    // ---- 3. 判定:超时 + 命中 ----
    for (auto & [bname, b] : bullets_) {
      if (b.delete_requested) {continue;}
      if (sim - b.born_sim >= bullet_lifetime_) {
        b.delete_requested = true;
        request_delete(bname);
        continue;
      }
      if (!b.has_judged_prev) {
        b.judge_x = b.cur_x;
        b.judge_y = b.cur_y;
        b.has_judged_prev = true;
        continue;  // 首拍只建立判定基准
      }
      for (auto & [tname, t] : tanks_) {
        if (!t.alive || !t.pos_valid) {continue;}
        if (tank_nodes::DamageCalculator::hit(
            bname, tname, b.judge_x, b.judge_y, b.cur_x, b.cur_y,
            t.x, t.y, hit_radius_))
        {
          t.health = tank_nodes::DamageCalculator::apply(t.health, 25.0);
          RCLCPP_INFO(get_logger(), "命中!%s → %s,剩余血量 %.0f",
            bname.c_str(), tname.c_str(), t.health);
          if (t.health <= 0.0) {
            t.alive = false;
            request_delete(tname);
            RCLCPP_WARN(get_logger(), "%s 被击毁!", tname.c_str());
          }
          b.delete_requested = true;
          request_delete(bname);
          break;
        }
      }
      if (!b.delete_requested) {  // 未命中 → 滚动判定基准
        b.judge_x = b.cur_x;
        b.judge_y = b.cur_y;
      }
    }

    publish_status();
  }

  static bool is_tank_name(const std::string & n)
  {
    return n == "player" || n.rfind("enemy_", 0) == 0;
  }

  void request_delete(const std::string & name)
  {
    if (!delete_cli_->service_is_ready()) {return;}
    auto req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    req->name = name;
    delete_cli_->async_send_request(req,
      [this, name](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture fut) {
        if (!fut.get()->success) {
          RCLCPP_WARN(get_logger(), "删除 %s 失败:%s",
            name.c_str(), fut.get()->status_message.c_str());
        }
      });
  }

  void publish_status()
  {
    for (const auto & [name, t] : tanks_) {
      tank_msgs::msg::TankStatus s;
      s.tank_name = name;
      s.is_player = t.is_player;
      s.health = t.health;
      s.alive = t.alive;
      status_pub_->publish(s);
    }
  }

  uint8_t game_state_ = tank_msgs::msg::GameState::RUNNING;
  double default_health_ = 100.0;
  double hit_radius_ = 0.5;
  double bullet_lifetime_ = 3.0;
  double judge_period_ = 0.05;
  double last_judge_sim_ = 0.0;

  std::map<std::string, TankEntry> tanks_;
  std::map<std::string, BulletEntry> bullets_;

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<tank_msgs::msg::GameState>::SharedPtr game_state_sub_;
  rclcpp::Publisher<tank_msgs::msg::TankStatus>::SharedPtr status_pub_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_cli_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CombatSystemNode>(rclcpp::NodeOptions()));
  return 0;
}
