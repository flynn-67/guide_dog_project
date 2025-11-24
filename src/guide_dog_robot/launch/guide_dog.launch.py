import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부를 변수로 받음 (기본값 True)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Webots + Nav2 실행 (Turtlebot3)
    # webots_ros2_turtlebot 패키지의 robot_launch.py를 실행
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('webots_ros2_turtlebot'), 'launch', 'robot_launch.py'
            )
        ]),
        launch_arguments={'nav': 'true', 'use_sim_time': 'true'}.items(),
    )

    # 2. 우리가 만든 Python BT 노드 실행
    # 중요: 여기서 use_sim_time을 True로 설정해줘야 시뮬레이터 시간과 동기화됨
    bt_node = Node(
        package='guide_dog_robot',
        executable='main_bt_node',
        name='guide_dog_bt_brain',
        output='screen',
        parameters=[{'use_sim_time': True}]  # <--- 이 부분이 핵심 해결책!
    )

    return LaunchDescription([
        turtlebot_launch,
        bt_node
    ])