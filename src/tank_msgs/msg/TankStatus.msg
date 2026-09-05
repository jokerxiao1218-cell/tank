# 坦克血量状态(combat_system 节点发布,每辆坦克一条)
string tank_name   # 坦克名:player / enemy_1 / enemy_2 / enemy_3
bool is_player     # 玩家还是敌人
float64 health     # 当前血量 0~100
bool alive         # 是否存活(死后 combat_system 不再销毁模型以外的残留)
