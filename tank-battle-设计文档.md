# tank-battle(ROS2 3D 坦克大战)设计文档

> 创建时间:2026-09-05 | 状态:设计中(待用户审)

## 1. 需求卡(已确认)

- **背景**:全新项目,在 `~/tank-battle` 从零建 ROS2 工作空间。目的:通过一个能玩的游戏,把 ROS2 通信机制(话题/服务)在完整分布式系统里用起来,顺便熟悉 Gazebo。
- **目标**:Gazebo 3D 场景里,键盘控制坦克移动/转炮塔/开炮;敌方坦克 AI 追击玩家并开炮;地图随机刷强化道具,捡到生效;支持开始/暂停;一个关卡:消灭全部敌人过关,自己血量归零失败。
- **输入/输出**:
  - 键位:`W/S` 前进后退 | `A/D` 车体转向 | `Q/E` 炮塔左右转 | `空格` 开炮 | `回车` 开始游戏 | `P` 暂停/恢复 | `Ctrl+C` 退出
  - 画面:Gazebo 窗口看 3D(鼠标拖视角),游戏状态(血量/分数/暂停提示)在终端彩色文字 HUD
  - 数值:玩家血量 100,炮弹伤害 25(可强化);敌方 3 辆各 100 血;道具每 10s 随机刷 1 个、场上最多 5 个
  - 4 种道具:🔴 射速(冷却 1.5s→0.75s)| 💥 火力(伤害+10,叠加上限 60)| ⚡ 速度(移速+30%,叠加上限 90%)| 🛡 维修(回满血)
- **约束**:
  - 依赖:只新增 Gazebo Classic 11 + gazebo_ros_pkgs + ros2_control 等 apt 包,不引入游戏引擎
  - 通信:游戏所有逻辑节点之间**全部走 ROS2**(C++ rclcpp)——这是本项目核心学习点
  - 错误处理:暂停时一切冻结(子弹飞行、AI、道具刷新全停,不只是画面);炮弹 3 秒自动销毁防堆积;节点崩溃要有清晰报错,不许静默挂
  - 改动范围:只新建 `~/tank-battle`,不碰其他项目
- **验收标准**:规则节点单测全绿 + 实际运行全流程验证(清单见第 7 节)
- **本项目无实体硬件,"真机验证" = 实际运行游戏逐项手动验证**

## 2. 现场调查

- **环境**:Ubuntu 22.04 + ROS2 Humble(/opt/ros/humble)+ Python 3.10;colcon/cmake/g++/rosdep 齐全;磁盘 847G 空闲;Gazebo 待装(用户执行中)
- **坦克模型**(后台代理实际下载验证过,不是看搜索摘要):
  - 选定路线:zthanxx/tank_bot 履带底盘([github.com/zthanxx/tank_bot](https://github.com/zthanxx/tank_bot),MIT 协议,11 link/10 joint,STL 合计仅 252KB,多轮差速写法现成)+ **自加几何体炮塔炮管**(2 link/2 joint,原生圆柱,零 mesh 依赖)
  - 已验证坑:网上搜索结果里的"坦克 URDF 仓库"有多个实为 404 幻觉;调研产物在 /tmp/tank_urdf_search/candidates/(重启会丢,开工时把 zthanxx 的 meshes 拷进项目并注明来源与 MIT 协议)
  - 参考写法:Water-Turret-AGV 的 ROS2 插件标签格式(已验证 Humble 可用,gazebo_ros_pkgs 格式)
- **分层**(本项目全新,自建四层):模型描述(URDF/xacro)→ Gazebo 物理仿真 → 游戏逻辑(ROS2 节点)→ 交互(键盘/HUD)
- **语言**:C++(rclcpp),用户已确认

## 3. 设计方案

### 3.1 总体架构:节点图

```
                    ┌──────────────┐
   键盘输入 ───────► │ keyboard_node │─── /player/cmd_vel (Twist) ────► ┐
   (终端raw模式读键)  └──────────────┘─── /player/turret_cmd ─────────► │ player_tank_node
                        │  ▲                                            │ ──► Gazebo diff_drive 插件(底盘)
   开始/暂停服务调用 ────┘  └── /game_state(状态广播) ◄── game_master ──► │ ──► ros2_control(炮塔旋转)
                                                              ▲           │ ──► spawn_entity(生成炮弹)
                                                              │           ▼
              ┌───────────────┬─┴──────────┬───────────────────────────┐
              │               │            │                           │
      enemy_ai_node ×3   combat_system   powerup_manager          hud_node
      (巡逻/追击/开炮)     (血量/伤害结算)  (刷道具/拾取/上buff)     (终端彩色HUD)
              │               │            │
              └── 都订阅 /game_state:暂停/恢复时统一冻结行为,谁都不许偷跑 ──┘
```

### 3.2 节点清单(全部 C++ rclcpp)

| 节点 | 职责 | 关键订阅 | 关键发布/服务 |
|---|---|---|---|
| `game_master` | 游戏状态机:IDLE→RUNNING↔PAUSED→WIN/LOSE;判定胜负(敌全灭/玩家死) | /tank_status(战斗结果) | 服务 /game/start、/game/pause(std_srvs/Trigger);话题 /game_state(10Hz 广播);调 Gazebo /pause_physics、/unpause_physics |
| `keyboard_node` | 终端 raw 模式读键盘(独立线程,不阻塞);按住持续、松开清零 | 无 | /player/cmd_vel(Twist)、/player/turret_cmd(Float64)、/player/fire(Empty);调 /game/start、/game/pause 服务 |
| `player_tank_node` | 玩家坦克执行器:透传底盘速度;积分算炮塔角;开炮(带冷却);生成炮弹实体 | /player/cmd_vel、/player/turret_cmd、/player/fire、/game_state | /gazebo 的 set_entity_state(炮弹初速) |
| `enemy_ai_node`(×3) | 敌人 AI:巡逻(随机点)→玩家距离<20m 追击 →<15m 开炮(冷却 2s) | /game_state、玩家坦克位姿(get_entity_state) | 自己的 /enemy_N/cmd_vel、fire |
| `combat_system_node` | 20Hz 轮询炮弹↔坦克距离(<0.5m 判命中);扣血;击毁销毁模型;炮弹 3s 超时销毁 | /game_state | /tank_status(每个坦克血量/存活) |
| `powerup_manager_node` | 10s 定时随机位刷道具(场上≤5);玩家距离<1m 判拾取;应用 buff | /game_state、玩家位姿 | /powerup_collected(事件)、/gazebo spawn/delete |
| `hud_node` | 终端彩色 HUD(血条/分数/状态提示),订阅所有状态话题 | /game_state、/tank_status、/powerup_collected | 无 |

### 3.3 ROS2 通信设计(本项目核心,每种机制都有出场)

| 机制 | 用在哪 | 为什么 |
|---|---|---|
| **话题-标准消息**(Twist/Empty/Float64) | 操控、开炮、炮塔 | 全场广播、低延迟,操控类指令标配 |
| **话题-自定义消息**(tank_msgs 包) | GameState、TankStatus、PowerupEvent | 游戏状态是多字段结构,自定义消息是 ROS2 教学必练 |
| **服务**(std_srvs/Trigger) | /game/start、/game/pause | 开始/暂停是"请求-确认"语义,要返回成功与否,比话题稳 |
| **参数**(YAML 可改) | 血量/伤害/冷却/道具间隔/敌人数量等全部数值 | 改数值不用改代码重编译,launch 传参,ROS2 参数机制教学点 |
| **Gazebo 桥接** | diff_drive(底盘)、gazebo_ros2_control(炮塔)、spawn_entity(炮弹/道具)、set_entity_state(初速/位置) | ROS2↔物理引擎的桥,真实机器人仿真同款套路 |

### 3.4 包结构(4 个包,职责单一)

```
~/tank-battle/
├── tank_description/   # 模型描述包:URDF/xacro、道具和炮弹的 SDF、STL(zthanxx,MIT,注明来源)
│   ├── urdf/tank.urdf.xacro    # 参数化:name/颜色/炮塔初始角,玩家敌人共用一份
│   ├── sdf/bullet.sdf、powerup.sdf
│   └── meshes/
├── tank_msgs/          # 自定义消息包:GameState.msg、TankStatus.msg、PowerupEvent.msg
├── tank_nodes/         # 全部 C++ 节点 + 逻辑类 + 单元测试
│   ├── src/(节点) include/tank_nodes/(逻辑类)
│   └── test/(gtest)
└── tank_bringup/       # 启动包:launch 文件、battlefield.world(30×30m 围墙+障碍箱)、参数 YAML
```

**可测性设计(单测能立起来的关键)**:每个节点的"规则逻辑"抽成纯 C++ 类(不依赖 ROS 任何东西),如 `GameStateMachine`、`DamageCalculator`、`PowerupEffects`、`EnemyDecision`、`KeymapParser`;节点只是把 ROS 通信接到这些类上。**单测测逻辑类,连接层靠运行验证**——逻辑对不对看单测,通不通看实跑,两层分开。

### 3.5 关键技术决策(每条带为什么)

1. **底盘用 diff_drive 插件,炮塔用 gazebo_ros2_control**:底盘差速是成熟插件一行配置;炮塔单关节用 ros2_control 的 JointPositionController 是标准做法(顺带学会 ros2_control,真机器人标配)。为什么不全用 ros2_control?底盘差速控制器配置量大,对初学者项目没必要。
2. **命中判定用 20Hz 距离轮询,不用 Gazebo contact 传感器**:contact 传感器要多实体配置、调试难、开销大;距离判定逻辑透明、单测可覆盖(见测试计划)。炮弹 30m/s,20Hz 一拍 1.5m,阈值 0.5m 理论上会漏——**对策:命中检测用"线段扫过"法(上一拍位置连到这一拍,线段与坦克距离<0.5m 即命中)**,数学上不漏弹。这是逻辑类,单测全覆盖。
3. **暂停两层**:调 Gazebo /pause_physics(物理真停,子弹悬停)+ 广播 GameState=PAUSED(各节点丢指令,AI/道具计时全停)。只做物理暂停的话,节点定时器还在跑,道具会照刷——两层缺一不可。
4. **炮弹初速两步设**:spawn_entity 生成 → set_entity_state 设 twist。SpawnEntity 服务没有初速度字段,两次服务调用间隔约一拍,理论上有轻微顿挫;若实测观感差,备选方案:apply_body_wrench 加冲量。风险点,见第 8 节。
5. **坦克模型尺寸**:zthanxx 原始约 30cm 玩具尺寸,STL 直接 scale 放大 3 倍(约 1m 长),战场 30×30m 围墙 + 几个箱子障碍。所有数值都是参数,实跑时再调平衡。
6. **道具/炮弹是独立 Gazebo 实体**(SDF 小方块/小球),由 powerup_manager/combat_system 通过 spawn/delete 服务管理生命周期。

### 3.6 风险点与对策

| 风险 | 对策 |
|---|---|
| 炮弹初速两步设,可能顿挫 | 实跑观察;不行换 apply_body_wrench(设计变更走流程) |
| zthanxx 是 ROS1 插件格式 | 已验证:多轮差速标签与 ROS2 diff_drive 几乎一一对应,重写标签即可;mesh 全在仓库 |
| 暂停时节点定时器仍触发 | 所有定时器/订阅回调第一行查 GameState,非 RUNNING 直接 return |
| 炮弹名冲突/泄漏 | 唯一编号 bullet_N;3s 超时销毁;销毁失败记 error 日志并重试一次 |
| 多实体性能 | 坦克≤4+炮弹+道具≤5,总量几十,远低于 Gazebo 负担 |
| 键盘 raw 模式抢占终端 | keyboard_node 独占运行时说明;Ctrl+C 恢复终端属性(注册信号处理) |

## 4. 测试计划(请用户重点审:这就是需求的精确定义)

按"逻辑类"分组,全部 gtest(colcon test),数字全部具体:

**A. GameStateMachine(状态机)**
- A1:初始 → IDLE;IDLE 发 start → RUNNING(正常)
- A2:RUNNING 发 pause → PAUSED;PAUSED 再发 pause → RUNNING(恢复,同键切换)
- A3:IDLE 发 pause → 仍 IDLE 且返回失败(边界:没开始就暂停,无效)
- A4:RUNNING 收到"最后 1 名敌人死亡"→ WIN
- A5:RUNNING 收到"玩家死亡"→ LOSE
- A6:PAUSED 收到胜负事件 → 胜负状态生效(暂停时打死的最后一枪结算正确)
- A7:WIN/LOSE 后发 start → 拒绝,返回失败(终局不可再战)

**B. DamageCalculator(伤害)**
- B1:玩家炮弹(25 伤)打 100 血敌人 → 敌人血 75、存活
- B2:火力 buff×3(25+30=55 伤)打 100 血敌人 → 1 发剩 45、2 发死
- B3:第 4 发 25 伤炮弹打 100 血敌人 → 敌人死亡,血量归 0 不是负数(边界)
- B4:炮弹擦边(线段距离 0.49m)→ 命中;(0.51m)→ 不命中(命中阈值 0.5m 边界)
- B5:自己的炮弹打自己 → 不掉血(炮弹归属判断)

**C. PowerupEffects(道具)**
- C1:开炮冷却 1.5s,捡射速道具 → 0.75s(正常)
- C2:伤害 25,捡火力×3 → 55、×4 → 60 封顶不上 70(叠加上限)
- C3:移速 1.0m/s,捡速度×3 → 1.9m/s、×4 → 封顶 1.9(叠加上限)
- C4:血量 37,捡维修 → 100(正常)
- C5:满血捡维修 → 仍 100,不超(边界)
- C6:场上 5 个道具时刷新计时到点 → 不生成第 6 个(上限边界)

**D. EnemyDecision(AI 决策,纯逻辑)**
- D1:玩家距离 25m → 输出巡逻(正常远)
- D2:玩家距离 19m → 输出追击(阈值 20m 边界内侧)
- D3:追击中距离 14m → 开炮且冷却 2s 内第二拍不开(正常近+冷却)
- D4:PAUSED 状态输入任何距离 → 输出"不动不打"(暂停冻结)

**E. KeymapParser(键位映射,纯逻辑)**
- E1:按 w → 前进速度=1.0;松开 w → 0.0(按住/松开)
- E2:同时 w+a → 前进 1.0 且角速度=1.0(组合键)
- E3:按 p 在 IDLE → 无 start 调用(无效暂停)
- E4:回车在 RUNNING → 不重复 start(幂等)
- E5:未知键(如 z)→ 无任何输出(垃圾输入不崩)

**F. 运行验证(单测做不了的,进第 7 节清单)**:Gazebo 画面、操控手感、炮弹观感、AI 实际表现、暂停时子弹真的悬停、性能帧率。

**怎么跑**:`cd ~/tank-battle && colcon build && colcon test && colcon test-result --verbose` → 全部用例 Passed。

## 5. Batch 划分与进度(每个 batch 完都编译通过+测试绿+可运行)

| # | 目标(做完系统什么状态) | 涉及文件 | 验证方式 | 状态 |
|---|---|---|---|---|
| 1 | 骨架立起:4 个包空壳+自定义消息+编译通过;git 基线 | tank_msgs 全部、其余包骨架 | colcon build 绿;`ros2 interface show tank_msgs/GameState` 正常 | 待执行 |
| 2 | 坦克开进 Gazebo:zthanxx 模型入库(MIT 注明)+自加炮塔、battlefield.world、launch 一条命令启动,场景里看到坦克 | tank_description、tank_bringup | 启动 Gazebo 看到坦克炮塔;命令行手动发 cmd_vel 坦克会动、发炮塔命令炮塔会转 | 待执行 |
| 3 | 键盘操控:WASD 开车、QE 转炮塔 | keyboard_node、player_tank_node | 按键开车/转炮塔,松开即停;单测 E1-E5 绿 | 待执行 |
| 4 | 开炮打伤害:空格发炮弹、飞行、命中扣血、击毁消失、3s 自毁 | player_tank_node(开炮)、combat_system_node、bullet.sdf | 场景里炮弹飞、打敌人(手动放一个靶)掉血,打完消失;单测 B1-B5 绿 | 待执行 |
| 5 | 开始/暂停:Enter 前一切锁死,P 全冻结(子弹悬停) | game_master_node、各节点接 GameState | IDLE 按键无效;Enter 后能动;P 后物理+逻辑全停,再 P 恢复;单测 A1-A7、D4 绿 | 待执行 |
| 6 | 敌人上阵:3 个敌人巡逻/追击/开炮,全灭→WIN、玩家死→LOSE | enemy_ai_node、game_master 胜负判定 | 实际被 3 个敌人围攻;全灭显示 WIN;被耗死显示 LOSE;单测 D1-D3 绿 | 待执行 |
| 7 | 道具系统:随机刷新、拾取、4 种 buff 生效 | powerup_manager_node、powerup.sdf | 捡到 4 种道具各自生效(射速/伤害/速度/回血);单测 C1-C6 绿 | 待执行 |
| 8 | HUD+收尾:终端彩色 HUD(血条/分数/状态),全流程打磨 | hud_node、参数文件整理 | 一条命令启动→完整玩一局到胜负;文档状态改已交付 | 待执行 |

状态只用:待执行 / 进行中 / ✅ / ❌ 卡住

## 6. 执行记录

| 日期 | Batch | 结果 | 备注(Commit/问题) |
|---|---|---|---|
| 2026-09-05 | 前期 | 环境调查、URDF 调研(3 候选实测)、需求卡+技术路线用户确认 | URDF 调研报告全文见会话记录 |

## 7. 真机验证清单(= 实际运行清单,用户执行逐项勾)

- [ ] **环境**:Gazebo 装好(`gazebo --version` 出 11.x);`cd ~/tank-battle && colcon build` 无报错
- [ ] **启动**:一条命令(`ros2 launch tank_bringup game.launch.py`)起全套:Gazebo 场景 + 4 辆坦克(3 敌 1 玩家)+ HUD 显示"按回车开始"
- [ ] **IDLE 锁**:按 W/A/D/Q/E/空格 → 坦克纹丝不动(没开始就该锁)
- [ ] **开始**:回车 → HUD 变 RUNNING,坦克能动了
- [ ] **操控**:W 前进 S 后退 A/D 转向流畅;Q/E 炮塔平滑转,松开即停
- [ ] **开炮**:空格 → 炮弹沿炮管方向飞出;1.5s 内连按第二发不会出弹(冷却);敌人被击中掉血、4 发死、尸体消失
- [ ] **暂停**:P → 子弹悬停在空中、敌人坦克全停、道具不刷;再按 P → 一切从冻结处继续(不是重开)
- [ ] **敌人 AI**:敌人巡逻;靠近 20m 内掉头朝我来;15m 内开始开炮打我
- [ ] **道具**:地上出现彩色道具;开过去碰到 → 消失 + HUD 显示获得;射速/火力/速度/维修各捡一次,效果如需求卡数值
- [ ] **胜负**:杀光 3 个敌人 → HUD 显示 WIN;故意站着挨打到死 → 显示 LOSE
- [ ] **异常路径 1**:暂停状态狂按空格/方向键 → 不发弹不移动;恢复后立刻正常(暂停丢单不被卡住)
- [ ] **异常路径 2**:炮弹打墙上/天上飞 3 秒 → 自动消失,不堆积
- [ ] **异常路径 3**:某坦克击毁后,相关话题里它的状态是"已死亡",节点不刷错误、不僵尸

## 8. 遗留风险与未验证点

- 炮弹初速两步设的顿挫感(3.5 节第 4 条),待 batch 4 实跑观察
- AI 只做"追击+开炮",不做避障/走位,实战可能显得憨——属于需求范围内接受,记录于此
- 坦克碰撞翻车(Gazebo 物理常见喜剧):靠降低重心+降低车速规避,batch 2 实跑调
- 多实体同时 spawn 的极端并发(玩家+3 敌同一拍全开炮)理论可行,未压测
