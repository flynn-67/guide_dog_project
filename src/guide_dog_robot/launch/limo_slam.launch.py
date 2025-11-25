from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('guide_dog_robot')

    # slam_toolbox 파라미터 파일
    slam_params_file = os.path.join(
        pkg_share,
        'config',
        'slam_params.yaml'
    )

    # 1) LIMO 기본 bringup
    # 👉 아직 LIMO 패키지 안 받아온 상태라 "예시"로만 둠.
    # 나중에 LIMO 공식 패키지 클론하면, 패키지/launch 이름만 맞춰서 수정하면 됨.
    #
    # 예를 들어, 나중에
    #   ros2 launch limo_bringup limo_start.launch.py
    # 이렇게 쓰라고 하면 ↓ 여기 package='limo_bringup', 'limo_start.launch.py'로 고치면 됨.

    limo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                # TODO: LIMO 공식 패키지 이름으로 바꾸기 (예: 'limo_bringup')
                get_package_share_directory('limo_bringup'),
                'launch',
                # TODO: 실제 bringup 파일 이름으로 바꾸기 (예: 'limo_start.launch.py')
                'limo_start.launch.py'
            )
        )
    )

    # 2) SLAM Toolbox 노드
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    # 필요하면 여기 RViz도 같이 띄워줄 수 있음 (나중에)
    # rviz_config_file = os.path.join(pkg_share, 'config', 'limo_slam.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_file]
    # )

    return LaunchDescription([
        limo_bringup_launch,
        slam_toolbox_node,
        # rviz_node,
    ])
