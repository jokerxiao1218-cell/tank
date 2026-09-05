"""tank-battle 一键启动:Gazebo 战场 + 坦克 + 游戏节点。

用法:
  ros2 launch tank_bringup game.launch.py            # 带画面
  ros2 launch tank_bringup game.launch.py gui:=false  # 无头模式(自动验证用)

B6 里程碑修复(多坦克插件错拍):gazebo_ros factory 的 spawn 服务存在
系统级 bug——模型 N 加载的插件 namespace 来自请求 N-1 的 SDF(滞后一拍,
实测连单客户端严格串行也复现;而 URDF 文件、话题内容、rsp 参数全部
100% 正确)。规避:坦克不走 factory spawn,改为 launch 生成期 xacro→
URDF→gz sdf 离线转 SDF,直接静态嵌入 world;gzserver 延迟 3s 启动,
让 rsp 先就绪(gazebo_ros2_control 插件要找它的 robot_description)。
factory 本身保留(world 插件),供炮弹/道具 spawn 用——它们无插件,
不受此 bug 影响。
"""
import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 坦克清单:(前缀, 车体色, 出生坐标)。
# 敌人出生约束:①围墙内(|x|,|y| < 15,B6 实测出生出墙的敌人一头撞墙卡死)
# ②掩体外(掩体在 ±8 环带内)③距玩家 15~20m(追击阈值 20 内立刻追击,
# 开炮圈 15 外留缓冲)
TANKS = [
    ('player', 'Gazebo/Green', 0.0, 0.0),
    ('enemy_1', 'Gazebo/Red', 13.0, 9.0),
    ('enemy_2', 'Gazebo/Red', -12.0, 9.0),
    ('enemy_3', 'Gazebo/Red', 10.0, -12.0),
]


def build_full_world(xacro_file, world_template, out_path, mesh_dir):
    """生成期组装完整 world:模板(墙/掩体/插件)+ 4 辆坦克静态模型。

    每辆坦克:xacro 出 URDF → gz sdf 离线转 SDF → 取 <model> 块、
    注入出生 <pose>、嵌入 </world> 前。
    """
    blocks = []
    for prefix, color, x, y in TANKS:
        urdf_file = f'/tmp/tank_battle_{prefix}.urdf'
        subprocess.run(
            ['xacro', xacro_file, f'prefix:={prefix}',
             f'body_color:={color}', '-o', urdf_file],
            check=True)
        sdf_txt = subprocess.run(
            ['gz', 'sdf', '-p', urdf_file],
            check=True, capture_output=True, text=True).stdout
        # B8 GUI 修复:gazebo_ros 把 URDF 的 package:// 转成 model://,
        # 而 gzclient(GUI)解析 model:// 需要 GAZEBO_MODEL_PATH 精确配置——
        # 我们环境没配,GUI 一启动就死循环找 mesh 拖到"无响应"(无头模式
        # 不加载视觉网格,故 8 批验证全绿没暴露)。改成 file:// 绝对路径,
        # 指向 install 里真实文件,任何启动方式都稳。
        sdf_txt = sdf_txt.replace(
            'model://tank_description/meshes/', 'file://' + mesh_dir + '/')
        m = sdf_txt[sdf_txt.find('<model'):sdf_txt.rfind('</model>') + len('</model>')]
        # model 名改为 prefix:URDF 的 robot name 是固定的(如 'tank'),
        # 4 辆同名 model 嵌入同一 world 会报 Non-unique names
        m = re.sub(r"<model name='[^']*'", f"<model name='{prefix}'", m, count=1)
        # 出生位姿:URDF 无全局位姿,转换后 model 也没有,作为首个子标签注入
        m = re.sub(r'(<model[^>]*>)',
                   r'\g<1><pose>{} {} 0 0 0 0</pose>'.format(x, y), m, count=1)
        blocks.append(m)
    with open(world_template) as f:
        world_txt = f.read()
    with open(out_path, 'w') as f:
        f.write(world_txt.replace('</world>', ''.join(blocks) + '\n</world>'))


def tank_ros_nodes(urdf_file, prefix, index):
    """一辆坦克的 ROS 侧节点:rsp + 两个控制器 spawner。

    坦克实体已静态嵌入 world(见模块 docstring),这里不再有 spawn 节点。
    spawner 按 index 错峰(6s + 1.5s×序号):4 个 controller_manager 同时
    冷启动会撞 class loader 竞态(B8 实测 player 的炮塔控制器偶发
    "no factory exists" 加载失败),错峰加载稳定。
    """
    period = 6.0 + 1.5 * index
    with open(urdf_file) as f:
        robot_description = f.read()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=prefix,
             parameters=[{'robot_description': robot_description}]),
        TimerAction(period=period, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', f'/{prefix}/controller_manager'])]),
        TimerAction(period=period + 0.5, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['turret_position_controller',
                       '--controller-manager', f'/{prefix}/controller_manager'])]),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('tank_bringup')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    tank_desc_share = get_package_share_directory('tank_description')

    xacro_file = os.path.join(tank_desc_share, 'urdf', 'tank.urdf.xacro')
    world_template = os.path.join(bringup_share, 'worlds', 'battlefield.world')
    full_world = '/tmp/tank_battle_full.world'
    mesh_dir = os.path.join(tank_desc_share, 'meshes')
    build_full_world(xacro_file, world_template, full_world, mesh_dir)

    entities = []
    for i, (prefix, color, x, y) in enumerate(TANKS):
        entities += tank_ros_nodes(f'/tmp/tank_battle_{prefix}.urdf', prefix, i)

    # 每辆敌人坦克两个节点:AI 决策(cmd_vel/turret_cmd/fire)+ 执行器(炮
    # 塔+开炮)。执行器复用 player_tank_node(prefix 参数化,弹名自动带 prefix)
    enemy_nodes = []
    for prefix, color, x, y in TANKS:
        if prefix == 'player':
            continue
        enemy_nodes += [
            Node(package='tank_nodes', executable='enemy_ai_node',
                 parameters=[{'prefix': prefix, 'use_sim_time': True}]),
            Node(package='tank_nodes', executable='player_tank_node',
                 parameters=[{'prefix': prefix}]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 画面'),
        # rsp 先就绪(0s),gzserver 延迟 3s:静态坦克的 gazebo_ros2_control
        # 插件加载时要向 /<prefix>/robot_state_publisher 要 robot_description
        *entities,
        TimerAction(period=3.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': full_world,
                              'gui': LaunchConfiguration('gui')}.items())]),
        # 游戏节点:总控(开始/暂停)、键盘、玩家执行器(开炮)、战斗系统(命中判定)
        Node(package='tank_nodes', executable='game_master_node', output='screen'),
        Node(package='tank_nodes', executable='keyboard_node',
             parameters=[{'prefix': 'player'}], output='screen'),
        Node(package='tank_nodes', executable='player_tank_node',
             parameters=[{'prefix': 'player'}], output='screen'),
        Node(package='tank_nodes', executable='combat_system_node',
             output='screen'),
        Node(package='tank_nodes', executable='powerup_manager_node',
             output='screen'),
        Node(package='tank_nodes', executable='hud_node', output='screen'),
        # 敌人:AI 决策 + 执行器(上面循环生成)
        *enemy_nodes,
    ])
