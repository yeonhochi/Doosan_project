#####################
import sys
from playsound import playsound
import rclpy
import threading
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from functools import partial
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
###################

################## 인터페이스 패키지
from project3_interfaces.msg import TableCommand  
##################

################## gui
from PySide2.QtCore import Qt
from PySide2.QtGui import QPixmap
from PySide2.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QPushButton, QTextBrowser
##################


class WaypointNavigator(Node):
    def __init__(self, gui):
        super().__init__('waypoint_navigator')
        self.gui = gui
        self.current_table = None
        self.pending_tables = []  # 대기열을 저장할 리스트

        # QoS 정책 설정 및 구독 초기화
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            TableCommand, 'to_robot', self.table_number_callback, qos_profile
        )
        
        # ActionClient 초기화
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 테이블 위치 정의
        self.waypoints = {
            "1": self.create_pose(3.664528328172522, 1.2609875925312812, 0.6398567973025034, 0.7684941632477003),
            "2": self.create_pose(3.6206477878016208, 0.7391463200202791, -0.7116280383994683, 0.7025564283128615),
            "3": self.create_pose(3.6102395878770883, -0.32800154280328186, -0.4873182591752054,0.8732244352252446),
            "4": self.create_pose(2.58119983414975, 1.5035286334267473, -0.9995238635463126, 0.030855245930832376),
            "5": self.create_pose(2.542072833175258, 0.44090932116091597, -0.99944958356542, 0.033174235649196915),
            "6": self.create_pose(2.5866782472939067, -0.5453673465009153, -0.9969975832510041, 0.07743267392811198),
            "7": self.create_pose( 1.3804259488730453,1.3094726692775238, 0.39033200573810595, 0.9206741689091028),
            "8": self.create_pose(1.4480420057507586, 0.9008003183975954, -0.4464297427305647, 0.8948186882299239),
            "9": self.create_pose(1.4303387442876556, -0.22721887656026704, -0.5007100003428423, 0.8656150966547839),
            "kitchen": self.create_pose(0.037669698473239, 0.0011686673289900305,  -0.0018310667849687275, 0.9999983235958093),
        }

    def create_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def table_number_callback(self, msg):
        table_number = msg.to_robot

        #INFO 로그 (일반적인 정보 기록)
        self.get_logger().info(f"Received table number: {table_number}")
        
        #WARN 로그 추가 (예상치 못한 상황)
        if table_number not in self.waypoints:
            self.get_logger().warn(f"Received an unknown table number: {table_number}")
            return

        if self.current_table is None:
            # 현재 작업 중인 테이블이 없으면 바로 처리
            self.current_table = table_number
            self.gui.update_image(table_number)  # Update the image in the GUI
            self.gui.add_log(f"{table_number}번 테이블로 이동 중입니다.", "INFO")
            playsound('/home/yeonho/project3_ws/src/project3/project3/서빙 시작합니다.m4a')
            self.send_goal(self.waypoints[table_number])
        else:
            # 현재 작업 중이면 대기열에 추가
            self.pending_tables.append(table_number)
            self.gui.add_log(f"{table_number}번 테이블이 대기 중입니다.", "INFO")

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"로봇 이동 진행 중: {feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.current_table = None
            self.get_logger().error("Goal rejected") #ERROR 로그 (실패한 경우)
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4:  # GoalStatus.STATUS_SUCCEEDED
            if self.current_table == None:
                self.gui.add_log(f"주방에 도착했습니다.", "INFO")
            else:   
                self.gui.add_log(f"{self.current_table}번 테이블 도착했습니다.", "INFO")
            playsound('/home/yeonho/project3_ws/src/project3/project3/주문하신 음식이 도착했습니다.m4a')
        else:
            pass

        # 현재 테이블 작업 완료, 대기열에서 다음 테이블을 처리
        self.current_table = None
        if self.pending_tables:
            next_table = self.pending_tables.pop(0)
            self.current_table = next_table
            self.gui.update_image(next_table)  # Update the image in the GUI
            self.gui.add_log(f"{next_table}번 테이블로 이동 중입니다.", "INFO")
            self.send_goal(self.waypoints[next_table])


class GUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.image_paths = {
            "1": "/home/yeonho/project3_ws/src/project3/project3/img/s1.png",
            "2": "/home/yeonho/project3_ws/src/project3/project3/img/s2.png",
            "3": "/home/yeonho/project3_ws/src/project3/project3/img/s3.png",
            "4": "/home/yeonho/project3_ws/src/project3/project3/img/s4.png",
            "5": "/home/yeonho/project3_ws/src/project3/project3/img/s5.png",
            "6": "/home/yeonho/project3_ws/src/project3/project3/img/s6.png",
            "7": "/home/yeonho/project3_ws/src/project3/project3/img/s7.png",
            "8": "/home/yeonho/project3_ws/src/project3/project3/img/s8.png",
            "9": "/home/yeonho/project3_ws/src/project3/project3/img/s9.png",
            "kitchen": "/home/yeonho/project3_ws/src/project3/project3/img/smile.png",
        }
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle("Serving Robot")
        self.setGeometry(100, 100, 500, 700)

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)

        # Kitchen Button
        kitchen_button = QPushButton("음식 받으셨으면, 터치해주세요!")
        kitchen_button.setFixedSize(500, 50)  # 버튼의 크기를 200x50으로 고정
        kitchen_button.setStyleSheet("color: red; background-color: white; font-size: 22px; font-family: Verdana; font-weight: bold;")
        kitchen_button.clicked.connect(partial(self.go_to_table, "kitchen"))
        layout.addWidget(kitchen_button)

        # Image display
        self.image_display = QLabel()
        self.image_display.setFixedSize(500, 500)
        self.image_display.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.image_display)

        # Logging window
        self.log_browser = QTextBrowser()
        self.log_browser.setFixedSize(480, 100)
        layout.addWidget(self.log_browser)

        self.setCentralWidget(central_widget)

    def go_to_table(self, table_number):
        self.update_image(table_number)
        if table_number in self.node.waypoints:
            self.node.send_goal(self.node.waypoints[table_number])
            playsound('/home/yeonho/project3_ws/src/project3/project3/맛있게 드세요.m4a')
            self.add_log(f"주방으로 복귀 중입니다.","INFO")

    def update_image(self, table_number):
        if table_number in self.image_paths:
            pixmap = QPixmap(self.image_paths[table_number])
            pixmap = pixmap.scaled(400, 400, Qt.KeepAspectRatio)
            self.image_display.setPixmap(pixmap)

    def add_log(self, message, level="INFO"):
        levels = {
            "INFO": "color: green;",
            "WARN": "color: orange;",
            "ERROR": "color: red;"
        }
        style = levels.get(level, "color: black;")
        formatted_message = f"<span style='{style}'>{level}: {message}</span>"
        self.log_browser.append(formatted_message)


def main():
    rclpy.init()

    app = QApplication(sys.argv)
    gui = GUI(None)  # GUI 초기화 (노드 전달 전)
    navigator = WaypointNavigator(gui)  # GUI 객체 전달
    gui.node = navigator  # GUI에 노드 전달

    ros_thread = threading.Thread(target=rclpy.spin, args=(navigator,))
    ros_thread.start()

    gui.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()





