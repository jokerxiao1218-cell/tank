# 游戏状态广播(game_master 节点 10Hz 发布,所有节点订阅,暂停/恢复靠它同步)
uint8 IDLE    = 0   # 未开始:一切锁死,按操控键无效
uint8 RUNNING = 1   # 进行中
uint8 PAUSED  = 2   # 暂停:物理+逻辑全部冻结
uint8 WIN     = 3   # 玩家胜利(敌人全灭)
uint8 LOSE    = 4   # 玩家失败(血量归零)

uint8 state         # 当前状态,取上面常量之一
uint8 enemy_alive   # 剩余敌人数量(HUD 显示用)
