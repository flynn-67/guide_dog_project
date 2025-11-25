from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('guide_dog_robot')

    slam_params_file = os.path.join(
        pkg_share,
        'config',
        'slam_params.yaml'
    )

    # 1) 같은 패키지 안의 bringup launch 포함
    limo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share,
                'launch',
                'limo_bringup.launch.py'
            )
        )
    )

    # 2) slam_toolbox 노드
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    return LaunchDescription([
        limo_bringup_launch,
        slam_toolbox_node,
    ])

