"""tank-battle 一键启动:Gazebo 战场 + 坦克 + 游戏节点。

用法:
  ros2 launch tank_bringup game.launch.py            # 带画面
  ros2 launch tank_bringup game.launch.py gui:=false  # 无头模式(自动验证用)

坦克清单在 TANKS 中维护(batch 4 先放 1 个静止敌人靶,batch 6 扩为 3 个并接 AI)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def tank_entities(xacro_file, prefix, body_color, x, y):
    """生成一辆坦克所需的节点列表:rsp + spawn + 两个控制器。

    前缀 prefix 决定该坦克全部话题/节点命名空间(/player/...、/enemy_1/...)。
    """
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=', prefix,
                 ' body_color:=', body_color]),
        value_type=str)
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=prefix,
             parameters=[{'robot_description': robot_description}]),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', prefix,
                        '-topic', f'/{prefix}/robot_description',
                        '-x', str(x), '-y', str(y), '-z', '0.2']),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster',
                        '--controller-manager', f'/{prefix}/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['turret_position_controller',
                        '--controller-manager', f'/{prefix}/controller_manager']),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('tank_bringup')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    tank_desc_share = get_package_share_directory('tank_description')

    xacro_file = os.path.join(tank_desc_share, 'urdf', 'tank.urdf.xacro')
    world_file = os.path.join(bringup_share, 'worlds', 'battlefield.world')

    # 场上坦克:(前缀, 车体色, 出生坐标)。敌人 batch 6 接 AI
    tanks = [
        ('player', 'Gazebo/Green', 0.0, 0.0),
        ('enemy_1', 'Gazebo/Red', 5.0, 0.0),   # batch 4:静止靶
    ]

    entities = []
    for prefix, color, x, y in tanks:
        entities += tank_entities(xacro_file, prefix, color, x, y)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 画面'),
        # 1. Gazebo 仿真器(战场世界,world 里挂了 ros_state 插件供战斗系统读位置)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file,
                              'gui': LaunchConfiguration('gui')}.items()),
        *entities,
        # 2. 游戏节点:键盘、玩家执行器(开炮)、战斗系统(命中判定)
        Node(package='tank_nodes', executable='keyboard_node',
             parameters=[{'prefix': 'player'}], output='screen'),
        Node(package='tank_nodes', executable='player_tank_node',
             parameters=[{'prefix': 'player'}], output='screen'),
        Node(package='tank_nodes', executable='combat_system_node',
             output='screen'),
    ])
