import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped

class Navigator:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        # 초기 위치를 발행(Publish)할 퍼블리셔 생성
        self.initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    def set_initial_pose(self, x=0.0, y=0.0, w=1.0):
        """로봇의 초기 위치를 강제로 설정합니다."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = w
        # 공분산(정확도) 설정 (대략적으로 설정)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        self.node.get_logger().info(f'Setting Initial Pose to ({x}, {y})...')
        self.initial_pose_pub.publish(msg)

    def send_goal(self, x, y, w=1.0):
        """목표 지점으로 이동 명령을 보냅니다."""
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Nav2 Action Server not available!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = w

        self.node.get_logger().info(f'Navigating to ({x}, {y})...')
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        return True