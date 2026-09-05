"""tank-battle 一键启动:Gazebo 战场 + 玩家坦克 + 炮塔控制器。

用法:
  ros2 launch tank_bringup game.launch.py            # 带画面
  ros2 launch tank_bringup game.launch.py gui:=false  # 无头模式(自动验证用)
batch 6 会在本文件追加 3 辆敌方坦克的 spawn。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('tank_bringup')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    tank_desc_share = get_package_share_directory('tank_description')

    xacro_file = os.path.join(tank_desc_share, 'urdf', 'tank.urdf.xacro')
    world_file = os.path.join(bringup_share, 'worlds', 'battlefield.world')

    # 玩家坦克:xacro 生成 URDF(参数 prefix=player 决定话题命名空间)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=player']),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 画面'),
        # 1. Gazebo 仿真器(战场世界)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file,
                              'gui': LaunchConfiguration('gui')}.items()),
        # 2. 玩家坦克的 robot_state_publisher(广播 TF,gazebo_ros2_control 也从它取 URDF)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='player',
             parameters=[{'robot_description': robot_description}]),
        # 3. 把玩家坦克生成进战场
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'player',
                        '-topic', '/player/robot_description',
                        '-x', '0', '-y', '0', '-z', '0.2']),
        # 4. 炮塔控制器:关节状态广播 + 位置控制器
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster',
                        '--controller-manager', '/player/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['turret_position_controller',
                        '--controller-manager', '/player/controller_manager']),
    ])
