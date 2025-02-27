import rclpy
import DR_init
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from DR_common2 import posx  # posx() 함수 사용

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 400, 400

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

class CurrentPosSubscriber(Node):
    def __init__(self):
        super().__init__("current_pos_subscriber")
        self.current_pos = None  # 현재 위치 저장 변수
        self.subscription = self.create_subscription(
            Float64MultiArray, "/dsr01/msg/current_posx", self.current_pos_callback, 10
        )

    def current_pos_callback(self, msg):
        """ ROS2 토픽을 통해 현재 위치를 업데이트 """
        self.current_pos = [round(d, 3) for d in msg.data]  # 소수점 3자리 반올림

    def get_current_pose(self):
        """ 현재 위치를 posx() 형태로 반환 (없으면 None) """
        return posx(self.current_pos) if self.current_pos else None


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cup_pick_place", namespace=ROBOT_ID)
    pos_subscriber = CurrentPosSubscriber()  # ✅ 현재 위치 구독 노드 생성

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_BASE,
            set_digital_output,
            wait,
            trans,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            release_compliance_ctrl,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def wait_digital_input(sig_num):
        wait(0.5)  # 신호가 들어올 때까지 대기

    def release():
        """ 컵을 놓는 동작 """
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(1)

    def grip():
        """ 컵을 잡는 동작 """
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    # ✅ 기본 초기 위치 (JReady)
    JReady = [0, 0, 90, 0, 90, 0]
    a0 = np.array([435.698, -49.988, 85.0, 125.178, -178.475, 40.713])
    a1 = np.array([a0[0] - 80,    a0[1] + 40, a0[2], a0[3], a0[4], a0[5]])
    a2 = np.array([a0[0] - 80,    a0[1] - 40, a0[2], a0[3], a0[4], a0[5]])
    a3 = np.array([a0[0] - 160,   a0[1] + 80, a0[2], a0[3], a0[4], a0[5]])
    a4 = np.array([a0[0] - 160,   a0[1],      a0[2], a0[3], a0[4], a0[5]])
    a5 = np.array([a0[0] - 160,   a0[1] - 80, a0[2], a0[3], a0[4], a0[5]])
    a6 = np.array([a0[0] - 50,    a0[1],      a0[2], a0[3], a0[4], a0[5]])
    a7 = np.array([a0[0] - 130,   a0[1] - 40, a0[2], a0[3], a0[4], a0[5]])
    a8 = np.array([a0[0] - 130,   a0[1] + 40, a0[2], a0[3], a0[4], a0[5]])
    a9 = np.array([a0[0] - 100,   a0[1],      a0[2], a0[3], a0[4], a0[5]])
    pick_position = np.array([345.842, 211.44, 221.3, 15.07, 179.452, 14.563])

    # ✅ 로봇 설정
    set_tool("Tool Weight")
    set_tcp("GripperDA_v1")
    # ✅ 컵 좌표 리스트 생성
    cup_positions = []
    for i in [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9]:
        cup_position = i
        cup_positions.append(cup_position.tolist())  # ✅ 리스트 형태로 변환

    # ✅ 새로운 순서로 재배열
    ordered_indices1 = [0, 1, 2, 3, 4, 5]
    ordered_indices2 = [6, 7, 8]
    ordered_indices3 = [9]
    reordered_cup_positions1 = [cup_positions[i] for i in ordered_indices1]
    reordered_cup_positions2 = [cup_positions[i] for i in ordered_indices2]
    reordered_cup_positions3 = [cup_positions[i] for i in ordered_indices3]

    # ✅ Ready Position 이동
    print("Moving to Ready Position...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    release()
    x=0


    ################## 1 층 #####################
    for index, pose in enumerate(reordered_cup_positions1):

        pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
        # 1️⃣ 컵 뽑는 위치로 이동
        movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        x-=10.7
        # 2️⃣ Force Control 적용
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        # 3️⃣ Force 감지될 때까지 대기
        while not check_force_condition(DR_AXIS_Z, max=8):
            rclpy.spin_once(pos_subscriber)  # ✅ 최신 위치 업데이트
        release_compliance_ctrl()

        print("✅ Force 감지됨! → 현재 위치에서 Z축으로 13mm 상승")

        # ✅ 현재 위치 얻기 (force 감지 후)
        current_pose = None
        while current_pose is None:  # 최신 위치가 들어올 때까지 계속 업데이트
            rclpy.spin_once(pos_subscriber)
            current_pose = pos_subscriber.get_current_pose()

        # 4️⃣ 현재 위치에서 Z축으로 13mm 상승
        lifted_position = trans(current_pose, [0, 0, 13, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lifted_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 5️⃣ 컵을 놓기 (release)
        release()

        # 6️⃣ 다시 Z축 14mm 하강
        lowered_position = trans(current_pose, [0, 0, -16, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 7️⃣ 컵을 잡기 (grip)
        grip()

        #올림
        lowered_position = trans(current_pose, [0, 0, 150, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 8️⃣ 컵 배치할 위치로 이동 (높이 보정 후 이동)
        lift_pose = trans(posx(pose), [0, 0, 100, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        lift_pose2 = trans(posx(lift_pose.tolist()), [0, 0, -98, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose2, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 9️⃣ Force 적용 후 컵 배치
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=35):
            rclpy.spin_once(pos_subscriber)

        release_compliance_ctrl()
        release()
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)


    ################## 2 층 #####################
        # ✅ 컵을 1개씩 집어서 이동하는 과정
    for index, pose in enumerate(reordered_cup_positions2):
        print(f"Moving to Cup Index {ordered_indices2[index]}: {pose}")

        pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        x-=10.7

        # 2️⃣ Force Control 적용
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        # 3️⃣ Force 감지될 때까지 대기
        while not check_force_condition(DR_AXIS_Z, max=8):
            rclpy.spin_once(pos_subscriber)  # ✅ 최신 위치 업데이트
        release_compliance_ctrl()

        print("✅ Force 감지됨! → 현재 위치에서 Z축으로 13mm 상승")

        # ✅ 현재 위치 얻기 (force 감지 후)
        current_pose = None
        while current_pose is None:  # 최신 위치가 들어올 때까지 계속 업데이트
            rclpy.spin_once(pos_subscriber)
            current_pose = pos_subscriber.get_current_pose()

        # 4️⃣ 현재 위치에서 Z축으로 13mm 상승
        lifted_position = trans(current_pose, [0, 0, 13, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lifted_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 5️⃣ 컵을 놓기 (release)
        release()

        # 6️⃣ 다시 Z축 14mm 하강
        lowered_position = trans(current_pose, [0, 0, -16, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 7️⃣ 컵을 잡기 (grip)
        grip()
        lowered_position = trans(current_pose, [0, 0, 170, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 8️⃣ 컵 배치할 위치로 이동 (높이 보정 후 이동)
        lift_pose = trans(posx(pose), [0, 0, 135, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        lift_pose2 = trans(posx(lift_pose.tolist()), [0, 0, -41, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose2, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 9️⃣ Force 적용 후 컵 배치
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=35):
            rclpy.spin_once(pos_subscriber)

        release_compliance_ctrl()
        release()
        end_pose = trans(posx(lift_pose.tolist()), [0, 0, 50, 0, 0, 0], DR_BASE, DR_BASE)
        movel(end_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)


    ################## 3 층 #####################
        # ✅ 컵을 1개씩 집어서 이동하는 과정
    for index, pose in enumerate(reordered_cup_positions3):
        print(f"Moving to Cup Index {ordered_indices3[index]}: {pose}")

        #movej(JReady, vel=VELOCITY, acc=ACC)
        pick2=trans(posx(pick_position.tolist()), [0, 0, 100, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick2.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        x-=10.7

        # 2️⃣ Force Control 적용
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        # 3️⃣ Force 감지될 때까지 대기
        while not check_force_condition(DR_AXIS_Z, max=8):
            rclpy.spin_once(pos_subscriber)  # ✅ 최신 위치 업데이트
        release_compliance_ctrl()

        print("✅ Force 감지됨! → 현재 위치에서 Z축으로 13mm 상승")

        # ✅ 현재 위치 얻기 (force 감지 후)
        current_pose = None
        while current_pose is None:  # 최신 위치가 들어올 때까지 계속 업데이트
            rclpy.spin_once(pos_subscriber)
            current_pose = pos_subscriber.get_current_pose()

        # 4️⃣ 현재 위치에서 Z축으로 13mm 상승
        lifted_position = trans(current_pose, [0, 0, 13, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lifted_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 5️⃣ 컵을 놓기 (release)
        release()

        # 6️⃣ 다시 Z축 14mm 하강
        lowered_position = trans(current_pose, [0, 0, -16, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 7️⃣ 컵을 잡기 (grip)
        grip()
        lowered_position = trans(current_pose, [0, 0, 200, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 8️⃣ 컵 배치할 위치로 이동 (높이 보정 후 이동)
        lift_pose = trans(posx(pose), [0, 0, 235, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        lift_pose2 = trans(posx(lift_pose.tolist()), [0, 0, -41, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose2, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 9️⃣ Force 적용 후 컵 배치
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=35):
            rclpy.spin_once(pos_subscriber)

        release_compliance_ctrl()
        release()

        end_pose = trans(posx(lift_pose.tolist()), [0, 0, 80, 0, 0, 0], DR_BASE, DR_BASE)
        movel(end_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)


    ################## 4 층 #####################
        #✅ 컵을 1개씩 집어서 이동하는 과정
    for index, pose in enumerate(reordered_cup_positions3):
        print(f"Moving to Cup Index {ordered_indices3[index]}: {pose}")

        pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)

        position1 = posx([345.486, 182.979, 92.233, 90.256, -89.446, 93.421])
        movel(position1, vel=VELOCITY, acc=ACC, ref = DR_BASE)
        grip()
        

        lifted_position2 = trans(position1, [0, 0, 80, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lifted_position2, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        movel(posx([354.671, 20.933, 329.934, 99.825, -89.219, -87.658]), vel=VELOCITY, acc=ACC, ref = DR_BASE)
        
        movel(posx([339.737, -76.131, 309.885, 99.825, -89.185, -87.725]), vel=VELOCITY, acc=ACC, ref = DR_BASE)

        position2 = posx([339.737, -76.131, 299.885, 99.825, -89.185, -87.725])
        movel(position2, vel=VELOCITY, acc=ACC, ref = DR_BASE)

        release()

        end_pose = trans(position2, [0, 100, 0, 0, 0, 0], DR_BASE, DR_BASE)
        movel(end_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)


    rclpy.shutdown()


if __name__ == "__main__":
    main()
