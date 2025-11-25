from launch import LaunchDescription
# 나중에 Node, IncludeLaunchDescription 필요해지면 import 추가
# from launch_ros.actions import Node

def generate_launch_description():
    # TODO: 나중에 여기다가 LIMO 모터 드라이버, LiDAR, 카메라 노드 등 추가할 예정
    return LaunchDescription([
        # 예시)
        # Node(
        #     package='limo_driver',
        #     executable='limo_base_node',
        #     name='limo_base',
        #     output='screen',
        # ),
    ])
