import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class WaypointNavigator(Node):
    def __init__(self, namespace):
        super().__init__(f'{namespace}_waypoint_navigator')
        self.namespace = namespace
        self.action_client = ActionClient(self, NavigateToPose, f'/{self.namespace}/navigate_to_pose')
        self.detect_publisher = self.create_publisher(String, '/start_detect', 10)

        # 🚨 Warning 및 Found 토픽 구독
        self.warning_subscriber = self.create_subscription(String, '/emergency', self.warning_callback, 10)
        self.found_subscriber = self.create_subscription(String, '/found', self.found_callback, 10)

        # tb2는 tb1 위치 구독, tb1은 tb2 위치 구독
        if self.namespace == "tb2":
            self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/tb1/amcl_pose', self.tb1_pose_callback, 10)
        else:  # tb1인 경우
            self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/tb2/amcl_pose', self.tb2_pose_callback, 10)

        self.target_pose = None
        self.warning_received = False
        self.found_triggered = False

        # 기존 웨이포인트 설정 (일반 이동 시 사용)
        if self.namespace == "tb1":
            self.waypoints = [
                self.create_pose(-1.4733, 1.8377, 0.6783, 0.7347),
                self.create_pose(-1.5821, -1.3790, -0.7420, 0.6703),
                self.create_pose(-5.5341, -1.4719, -0.7261, 0.6874),
                self.create_pose(-5.6923, 1.6094, 0.6700, 0.7422),
                self.create_pose(-9.5801, 1.9556, 0.7883, 0.6151),
                self.create_pose(-9.9126, -1.1083, -0.7140, 0.7001),
            ]
        else:  # tb2
            self.waypoints = [
                self.create_pose(-1.429, -10.8080, 0.7002, 0.7138),
                self.create_pose(-1.7364, -13.9427, -0.6383, 0.7697),
                self.create_pose(-5.3562, -14.0792, -0.6841, 0.7294),
                self.create_pose(-5.8086, -10.7540, 0.6418, 0.7668),
                self.create_pose(-9.7270, -10.6888, 0.6247, 0.7808),
                self.create_pose(-9.5943, -13.8066, -0.6786, 0.7344),
            ]

        self.current_index = 0
        self.navigate_to_next_waypoint()

    def create_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def navigate_to_next_waypoint(self):
        if self.warning_received or self.found_triggered:
            return

        if self.current_index >= len(self.waypoints):
            self.get_logger().info(f"{self.namespace} -> 모든 웨이포인트 방문 완료! 다시 처음부터 시작합니다.")
            self.current_index = 0

        waypoint = self.waypoints[self.current_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        self.get_logger().info(f"{self.namespace} -> 웨이포인트 {self.current_index + 1} 이동 중: {waypoint.pose.position.x}, {waypoint.pose.position.y}")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("목표 거부됨")
            return

        self.get_logger().info(f"{self.namespace} - 목표 수락됨, 결과 대기 중...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error(f"{self.namespace} -> 이동 결과를 받지 못함! (result is None)")
            return

        if result.status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(f"{self.namespace} -> 목표 도착 완료!")
            msg = String()

            if hasattr(self, "emergency_mode") and self.emergency_mode:
                msg.data = self.emergency_detect_id
                self.get_logger().info(f"{self.namespace} -> 긴급 대피 위치 도착! /start_detect {msg.data} 발행")
                self.detect_publisher.publish(msg)
                self.emergency_mode = False
            elif getattr(self, "target_navigation", False):
                # tb1가 target navigation 모드일 경우, tb2 위치 도착 후 목표 이동 완료
                msg.data = "target"  # 필요에 따라 메시지 내용을 설정
                self.detect_publisher.publish(msg)
                self.get_logger().info(f"{self.namespace} -> /start_detect target navigation 완료")
                # target_navigation 플래그는 유지하거나, 이후 상황에 따라 재설정 가능
            else:
                if self.namespace == "tb1":
                    msg.data = str(self.current_index + 1)
                else:
                    msg.data = str(self.current_index + 7)
                self.detect_publisher.publish(msg)
                self.get_logger().info(f"{self.namespace} -> /start_detect 신호 발행! (웨이포인트: {msg.data})")

            time.sleep(5)
            # target_navigation 모드인 경우 일반 웨이포인트 순회를 재개하지 않음
            if not self.warning_received and not self.found_triggered and not getattr(self, "target_navigation", False) and not hasattr(self, "emergency_detect_id"):
                self.current_index += 1
                self.navigate_to_next_waypoint()


    def warning_callback(self, msg):
        self.get_logger().warn(f"{self.namespace} -> /warning 수신! 웨이포인트 순회 중단!")
        self.warning_received = True
        self.stop_navigation()
        self.navigate_to_emergency_location()

    def navigate_to_emergency_location(self):
        if self.namespace == "tb1":
            emergency_pose = self.create_pose(
                -9.568263569421633,  # tb1 긴급 위치 X
                0.6390489123349085,  # tb1 긴급 위치 Y
                -0.998124919511526,  # Orientation Z
                0.061209844388869096  # Orientation W
            )
            self.emergency_detect_id = "13"
            self.emergency_mode = True
        else:
            emergency_pose = self.create_pose(
                -9.568148290125094,  # tb2 긴급 위치 X
                -12.632184782139419,  # tb2 긴급 위치 Y
                -0.9990748863283317,  # Orientation Z
                0.04300431964385757  # Orientation W
            )
            self.emergency_detect_id = "14"
            self.emergency_mode = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = emergency_pose

        self.get_logger().info(f"{self.namespace} -> 긴급 대피 위치로 이동 중: {goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.emergency_goal_response_callback)

    def emergency_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("긴급 대피 목표가 거부되었습니다.")
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.emergency_get_result_callback)

    def emergency_get_result_callback(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error("긴급 대피 위치 결과 없음")
            return
        if result.status == 4:
            self.get_logger().info("긴급 대피 위치 도착 완료!")
            msg = String()
            if self.namespace == "tb1":
                msg.data = "13"
                self.detect_publisher.publish(msg)
                self.get_logger().info("14 전송 완료")
                # tb2에 대한 긴급 웨이포인트 동작 (예시)
                self.emergency_waypoints = [  
                    self.create_pose(-12.0959263569421, 0.6390489123349085, -0.998124919511526, 0.061209844388869096),
                    self.create_pose(-12.0959263569421, 0.6390489123349085, 0.8042427245247965, 0.5943009675651995),
                    self.create_pose(-12.0959263569421, 0.6390489123349085, -0.6006962700214279, 0.7994773237449225)
                ]
            else:  # tb1
                msg.data = "14"
                self.detect_publisher.publish(msg)
                self.get_logger().info("13 전송 완료")
                # tb1에 대한 긴급 웨이포인트 동작 (예시)
                self.emergency_waypoints = [
                    self.create_pose(-12.095926155628286, -12.247232381124249, 0.9958793609037424, 0.09068791830201794),
                    self.create_pose(-12.153843602123489, -12.247594021320014, 0.685489392837646, 0.7280826136552605),
                    self.create_pose(-12.152829852315874, -12.248162966670696, -0.6006962700214279, 0.7994773237449225)
                ]
            self.emergency_wp_index = 0
            self.navigate_next_emergency_waypoint()

                  

    def navigate_next_emergency_waypoint(self):
        # /found 메시지가 수신되어 found_triggered가 True라면, 긴급 웨이포인트 이동을 중단합니다.
        if self.found_triggered:
            self.get_logger().warn(f"{self.namespace} -> 긴급 웨이포인트 이동 중단됨 (/found 수신)")
            return

        if self.emergency_wp_index >= len(self.emergency_waypoints):
            self.get_logger().info("모든 긴급 웨이포인트 도착 완료!")
            return

        next_pose = self.emergency_waypoints[self.emergency_wp_index]
        next_goal = NavigateToPose.Goal()
        next_goal.pose = next_pose

        self.get_logger().info(f"{self.namespace} -> 긴급 웨이포인트 {self.emergency_wp_index + 1} 이동: {next_pose.pose.position.x}, {next_pose.pose.position.y}")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(next_goal)
        self._send_goal_future.add_done_callback(self.emergency_waypoint_goal_response_callback)


    def emergency_waypoint_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("긴급 웨이포인트 목표가 거부되었습니다.")
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.emergency_waypoint_get_result_callback)

    def emergency_waypoint_get_result_callback(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error("긴급 웨이포인트 결과 없음")
            return
        if result.status == 4:
            self.get_logger().info(f"긴급 웨이포인트 {self.emergency_wp_index + 1} 도착 완료!")
            self.emergency_wp_index += 1
            time.sleep(2)
            self.navigate_next_emergency_waypoint()

    def stop_navigation(self):
        self.get_logger().warn(f"{self.namespace} -> 현재 이동 중지 요청됨!")
        if hasattr(self, "_send_goal_future") and self._send_goal_future:
            if hasattr(self._send_goal_future, "cancel"):
                self._send_goal_future.cancel()
        if hasattr(self, "_get_result_future") and self._get_result_future:
            if hasattr(self._get_result_future, "cancel"):
                self._get_result_future.cancel()
        self.get_logger().warn(f"{self.namespace} -> 목표 이동이 취소되었습니다.")

    def found_callback(self, msg):
        if self.found_triggered:
            self.get_logger().warn(f"{self.namespace} -> /found 이미 처리됨. 무시함.")
            return

        if msg.data == "1":
            if self.namespace == "tb1":
                self.get_logger().warn(f"{self.namespace} -> /found 1 수신! 즉시 동작 멈춤!")
                self.found_triggered = True
                self.stop_navigation()
            elif self.namespace == "tb2":
                self.get_logger().warn(f"{self.namespace} -> /found 1 수신! tb1의 위치로 이동 시작!")
                self.found_triggered = True
                if self.target_pose:
                    self.navigate_to_target()
        elif msg.data == "2":
            if self.namespace == "tb2":
                self.get_logger().warn(f"{self.namespace} -> /found 2 수신! 즉시 동작 멈춤!")
                self.found_triggered = True
                self.stop_navigation()
            elif self.namespace == "tb1":
                self.get_logger().warn(f"{self.namespace} -> /found 2 수신! tb2의 위치로 이동 시작!")
                self.found_triggered = True
                self.target_navigation = True  # target navigation mode로 전환
                if self.target_pose:
                    self.navigate_to_target()


    def tb1_pose_callback(self, msg):
        self.target_pose = msg.pose.pose

    def tb2_pose_callback(self, msg):
        self.target_pose = msg.pose.pose

    def navigate_to_target(self):
        if self.target_pose is None:
            self.get_logger().error(f"{self.namespace} -> 목표 위치를 아직 수신하지 못함!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose = self.target_pose

        self.get_logger().info(f"{self.namespace} -> 목표 위치로 이동 중: {goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

def main():
    rclpy.init()
    tb1_navigator = WaypointNavigator("tb1")
    tb2_navigator = WaypointNavigator("tb2")

    executor = MultiThreadedExecutor()
    executor.add_node(tb1_navigator)
    executor.add_node(tb2_navigator)

    try:
        executor.spin()
    finally:
        tb1_navigator.destroy_node()
        tb2_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
