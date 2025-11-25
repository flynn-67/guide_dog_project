from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('guide_dog_robot')

    # 저장된 맵 (나중에 slam_toolbox로 save_map 한 후 이 경로에 옮겨 놓을 것)
    map_file = os.path.join(
        pkg_share,
        'config',
        'maps',
        'school_floor1.yaml'   # TODO: 실제 파일 이름으로 수정
    )

    # Nav2 파라미터 (일단 껍데기만 두고 나중에 채우기)
    nav2_params = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )

    # LIMO bringup (위와 같은 TODO)
    limo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('limo_bringup'),
                'launch',
                'limo_start.launch.py'
            )
        )
    )

    # slam_toolbox localization 모드 대신, 나중에 AMCL 쓸 수도 있음
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'map': map_file
        }.items()
    )

    return LaunchDescription([
        limo_bringup_launch,
        nav2_bringup,
    ])
