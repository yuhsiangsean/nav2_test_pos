"""
最小測試 launch（位置控制版）.

static TF(map->odom) + px4_odom_tf + position_controller（取代
cmd_vel_bridge + nav2_controller）+ 一組精簡過的 Nav2 節點。

跟 px4_nav2_bridge 的差異：controller_server 不是官方 nav2_controller 執行檔，
是 nav2_test_pos/position_controller.py 自己實作的 FollowPath action server，
直接送位置 setpoint 給 PX4，不經過 /cmd_vel 速度控制這條線，所以這裡也不需要
velocity_smoother（沒有速度可以平滑）。

不含 map_server / AMCL（無地圖、無 GPS，室內用 OptiTrack 或 PX4 local frame 當 map）。
所有節點 use_sim_time:=false。

這裡沒有直接 include nav2_bringup 的 navigation_launch.py，原因跟 px4_nav2_bridge
一樣：collision_monitor 需要 /scan 才能正常工作，沒有 /scan 時它的必填陣列參數
會讓 rclcpp 直接 crash；route_server / docking_server 這個最小測試也用不到。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# /tf、/tf_static 相對名稱重映射，跟 nav2_bringup 的 navigation_launch.py 一致
TF_REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav2_test_pos')
    default_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    target_alt = LaunchConfiguration('target_alt')
    topic_odometry = LaunchConfiguration('topic_odometry')
    topic_offboard_mode = LaunchConfiguration('topic_offboard_mode')
    topic_setpoint = LaunchConfiguration('topic_setpoint')
    topic_command = LaunchConfiguration('topic_command')
    autostart = LaunchConfiguration('autostart')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='nav2_params.yaml 路徑')
    declare_target_alt = DeclareLaunchArgument(
        'target_alt', default_value='1.0',
        description='position_controller 的飛行高度 [m]')
    declare_topic_odometry = DeclareLaunchArgument(
        'topic_odometry', default_value='/fmu/out/vehicle_odometry',
        description='PX4 vehicle_odometry topic（命名空間不同時要覆寫，例如 /MAV4/fmu/out/vehicle_odometry）')
    declare_topic_offboard_mode = DeclareLaunchArgument(
        'topic_offboard_mode', default_value='/fmu/in/offboard_control_mode',
        description='PX4 offboard_control_mode topic')
    declare_topic_setpoint = DeclareLaunchArgument(
        'topic_setpoint', default_value='/fmu/in/trajectory_setpoint',
        description='PX4 trajectory_setpoint topic')
    declare_topic_command = DeclareLaunchArgument(
        'topic_command', default_value='/fmu/in/vehicle_command',
        description='PX4 vehicle_command topic')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='lifecycle_manager 是否自動 configure/activate')

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    px4_odom_tf = Node(
        package='nav2_test_pos',
        executable='px4_odom_tf',
        parameters=[{'topic_odometry': topic_odometry}],
    )

    # 取代 nav2_controller 的 controller_server + cmd_vel_bridge：
    # 自己實作 FollowPath action server，直接位置控制送給 PX4。
    controller_server = Node(
        package='nav2_test_pos',
        executable='position_controller',
        output='screen',
        parameters=[{
            'topic_odometry': topic_odometry,
            'topic_offboard_mode': topic_offboard_mode,
            'topic_setpoint': topic_setpoint,
            'topic_command': topic_command,
            'target_alt': target_alt,
        }],
    )
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': autostart, 'node_names': LIFECYCLE_NODES}],
    )

    return LaunchDescription([
        declare_params_file,
        declare_target_alt,
        declare_topic_odometry,
        declare_topic_offboard_mode,
        declare_topic_setpoint,
        declare_topic_command,
        declare_autostart,
        static_tf_map_odom,
        px4_odom_tf,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])
