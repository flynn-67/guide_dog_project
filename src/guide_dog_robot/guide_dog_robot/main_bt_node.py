import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from guide_dog_robot.behaviors.navigation import Navigator
import time

class GuideDogBrain(Node):
    def __init__(self):
        super().__init__('guide_dog_brain')
        
        # [중요] 시뮬레이션 시간 사용 강제 설정
        self.declare_parameter('use_sim_time', True)
        
        self.get_logger().info("--- 멍멍! 안내견 로봇 두뇌 가동 시작 ---")
        
        self.navigator = Navigator(self)
        
        # Nav2가 완전히 켜질 때까지 넉넉하게 기다립니다 (10초)
        self.get_logger().info("시스템 초기화 대기 중 (10초)...")
        self.init_timer = self.create_timer(10.0, self.set_init_pose)
        self.mission_started = False

    def set_init_pose(self):
        self.init_timer.cancel() # 타이머 정지
        
        # 1. 초기 위치 설정 (0, 0)
        # 로그에 [Navigator] 초기 위치 설정 중... 이 떠야 함
        self.navigator.set_initial_pose(0.0, 0.0)
        
        # 2. 위치가 잡힐 때까지 5초 더 대기 후 출발
        self.get_logger().info("위치 초기화 완료. 5초 뒤 출발합니다.")
        self.move_timer = self.create_timer(5.0, self.start_mission)

    def start_mission(self):
        self.move_timer.cancel()
        
        if self.mission_started:
            return
            
        # 목표 지점: 앞으로 1.5미터
        target_x = 1.5
        target_y = 0.0
        
        self.get_logger().info(f"명령: 앞으로 가! (x={target_x})")
        self.navigator.send_goal(target_x, target_y)
        self.mission_started = True

def main(args=None):
    rclpy.init(args=args)
    node = GuideDogBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()