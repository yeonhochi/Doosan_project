import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 로봇 초기 위치
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = 'map'

        self.initial_pose.pose.pose.position.x = 0.037669698473239
        self.initial_pose.pose.pose.position.y = 0.0011686673289900305
        self.initial_pose.pose.pose.position.z = 0.0

        self.initial_pose.pose.pose.orientation.x = 0.0
        self.initial_pose.pose.pose.orientation.y = 0.0
        self.initial_pose.pose.pose.orientation.z = -0.0018310667849687275
        self.initial_pose.pose.pose.orientation.w = 0.9999983235958093

        # 'covariance'는 반드시 길이가 36인 리스트여야 함
        self.initial_pose.pose.covariance = [
            0.17223733445864653, -0.014496324008971379, 0.0, 0.0, 0.0, 0.0,
            -0.014496324008971375, 0.20639058046990733, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06149202015697064
        ]

        # 2초 동안 퍼블리시
        self.timer = self.create_timer(0.5, self.publish_initial_pose)  # 0.5초마다 퍼블리시
        self.shutdown_timer = self.create_timer(2.0, self.shutdown_node)  # 2초 뒤 노드 종료

    def publish_initial_pose(self):
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.initial_pose)
        self.get_logger().info('Published initial pose.')

    def shutdown_node(self):
        self.get_logger().info('Shutting down after 2 seconds.')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = InitialPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
