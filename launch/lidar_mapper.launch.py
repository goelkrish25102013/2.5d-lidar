import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_mapper_ros2')
    urdf_path = os.path.join(pkg_share, 'urdf', 'lidar_rig.urdf')

    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='wired',
        description='Connection mode: "wired" or "wireless"'
    )
    mode = LaunchConfiguration('mode')

    is_wired = PythonExpression(["'", mode, "' == 'wired'"])
    is_wireless = PythonExpression(["'", mode, "' == 'wireless'"])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    lidar_driver_wired = Node(
        package='lidar_mapper_ros2',
        executable='lidar_driver_node_wired',
        name='lidar_driver_node',
        output='screen',
        condition=IfCondition(is_wired)
    )

    pan_controller_wired = Node(
        package='lidar_mapper_ros2',
        executable='pan_controller_node_wired',
        name='pan_controller_node',
        output='screen',
        condition=IfCondition(is_wired)
    )

    lidar_driver_wireless = Node(
        package='lidar_mapper_ros2',
        executable='lidar_driver_node_wireless',
        name='lidar_driver_node',
        output='screen',
        condition=IfCondition(is_wireless)
    )

    pan_controller_wireless = Node(
        package='lidar_mapper_ros2',
        executable='pan_controller_node_wireless',
        name='pan_controller_node',
        output='screen',
        condition=IfCondition(is_wireless)
    )

    scan_accumulator_node = Node(
        package='lidar_mapper_ros2',
        executable='scan_accumulator_node',
        name='scan_accumulator_node',
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        robot_state_publisher_node,
        lidar_driver_wired,
        pan_controller_wired,
        lidar_driver_wireless,
        pan_controller_wireless,
        scan_accumulator_node,
    ])