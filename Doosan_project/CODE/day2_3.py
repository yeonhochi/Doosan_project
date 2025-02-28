import rclpy
import DR_init
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from DR_common2 import posx  # posx() 함수 사용

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 150, 150

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

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def wait_digital_input(sig_num):
        wait(sig_num)  # 신호가 들어올 때까지 대기

    def release():
        """ 컵을 놓는 동작 """
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(2, ON)
        wait_digital_input(1)

    def grip():
        """ 컵을 잡는 동작 """
        release()
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait_digital_input(1)

    def soft_grip():
        """ 컵을 잡는 동작 """
        release()
        set_digital_output(1, ON)
        wait_digital_input(1)

    # ✅ 기본 초기 위치 (JReady)
    JReady = [0, 0, 90, 0, 90, 0]

    # ✅ 컵을 뽑는 위치
    pick_position = np.array([328.351, 145.223, 222.923, 24.416, -179.317, 9.374])

    # ✅ 1번째 컵 (시작점)
    a1 = np.array([540.198, 183.427, 106.211, 93.84, -179.029, 93.375])

    # ✅ 15번째 컵 (끝점)
    a2 = np.array([166.595, -239.233, 117.477, 87.143, -178.995, 86.97])

    # ✅ 컵 개수 (1~15번째 → 총 15개)
    num_cups = 14


    # ✅ 로봇 설정
    set_tool("Tool Weight")
    set_tcp("GripperDA_v1")


# ✅ 컵 좌표 리스트 생성
    cup_positions = []
    for i in range(num_cups):
        alpha = i / (num_cups - 1)  # 0 ~ 1 사이 보간값
        cup_position = (1 - alpha) * a1 + alpha * a2  # 선형 보간
        cup_positions.append(cup_position.tolist())  # ✅ 리스트 형태로 변환



    # ✅ 새로운 순서로 재배열
    ordered_indices0 = [0,12]
    ordered_indices1 = [3, 5, 7, 9]
    ordered_indices2 = [4,6,8]
    ordered_indices3 = [5,7]  # ✅ 재배치된 순서
    ordered_indices4 = [6]  # ✅ 재배치된 순서
    reordered_cup_positions0 = [cup_positions[i] for i in ordered_indices0]
    reordered_cup_positions1 = [cup_positions[i] for i in ordered_indices1]
    reordered_cup_positions2 = [cup_positions[i] for i in ordered_indices2]
    reordered_cup_positions3 = [cup_positions[i] for i in ordered_indices3]
    reordered_cup_positions4 = [cup_positions[i] for i in ordered_indices4]

    # ✅ Ready Position 이동
    print("Moving to Ready Position...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    x=0


        ################## 0 층 #####################
    # ✅ 컵을 1개씩 집어서 이동하는 과정
    for index, pose in enumerate(reordered_cup_positions0):
        print(f"Moving to Cup Index {ordered_indices0[index]}: {pose}")
        release()  # 컵을 잡기 전 놓아둠
        #movej(JReady, vel=VELOCITY, acc=ACC)
        if index == 0:
            pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
            # 1️⃣ 컵 뽑는 위치로 이동
            movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            grip()
            
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

            reverse= posx([325.103 , 171.534, 200.81+x, 88.117, 72.436, 89.051])
            movel(reverse, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            # # 6️⃣ 다시 Z축 14mm 하강
            # lowered_position = trans(current_pose, [0, 0, -16, 0, 0, 0], DR_BASE, DR_BASE)
            # movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

            # 7️⃣ 컵을 잡기 (grip)
            soft_grip()

            #올림
            lowered_position = trans(reverse, [0, 0, 200, 0, 0, 0], DR_BASE, DR_BASE)
            movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            
            lift_pose = posx([547.454, 195.746, 15.796, 68.74, 109.63, -88.821])
            movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        

            release()

            lifted_position1 = trans(lift_pose, [0, -100, 0, 0, 0, 0], DR_BASE, DR_BASE)
            movel(lifted_position1, vel=VELOCITY, acc=ACC, ref=DR_BASE)

            lifted_position2 = trans(lift_pose , [0, -100, 300, 0, 0, 0], DR_BASE, DR_BASE)
            movel(lifted_position2, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            x-=10
        else:
            pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
            # 1️⃣ 컵 뽑는 위치로 이동
            movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            grip()
            
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

            reverse= posx([325.103 , 171.534, 200.81+x, 88.117, 72.436, 89.051])
            movel(reverse, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            # # 6️⃣ 다시 Z축 14mm 하강
            # lowered_position = trans(current_pose, [0, 0, -16, 0, 0, 0], DR_BASE, DR_BASE)
            # movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

            # 7️⃣ 컵을 잡기 (grip)
            soft_grip()

            #올림
            lowered_position = trans(reverse, [0, 0, 400, 0, 0, 0], DR_BASE, DR_BASE)
            movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            movej(JReady, vel=VELOCITY, acc=ACC)
            lift_pose = posx([168.042, -204.419, 15.665, 9.006, -106.117, 88.826])
            movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

            release()

            lifted_position1 = trans(lift_pose, [50, 0, 0, 0, 0, 0], DR_BASE, DR_BASE)
            movel(lifted_position1, vel=VELOCITY, acc=ACC, ref=DR_BASE)

            lifted_position2 = trans(lift_pose, [50, 0, 100, 0, 0, 0], DR_BASE, DR_BASE)
            movel(lifted_position2, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            movej(JReady, vel=VELOCITY, acc=ACC)
            x-=10

    ################## 1 층 #####################
    # ✅ 컵을 1개씩 집어서 이동하는 과정
    for index, pose in enumerate(reordered_cup_positions1):
        print(f"Moving to Cup Index {ordered_indices1[index]}: {pose}")
        release()  # 컵을 잡기 전 놓아둠
        #movej(JReady, vel=VELOCITY, acc=ACC)

        pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
        # 1️⃣ 컵 뽑는 위치로 이동
        movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        x-=10
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
        lowered_position = trans(current_pose, [0, 0, 200, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 8️⃣ 컵 배치할 위치로 이동 (높이 보정 후 이동)
        lift_pose = trans(posx(pose), [0, 0, 150, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        lift_pose2 = trans(posx(lift_pose.tolist()), [0, 0, -160, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose2, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 9️⃣ Force 적용 후 컵 배치
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=35):
            rclpy.spin_once(pos_subscriber)

        release_compliance_ctrl()
        release()
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    print("✅ 모든 컵을 배치 완료!")

################## 2 층 #####################
        # ✅ 컵을 1개씩 집어서 이동하는 과정
    for index, pose in enumerate(reordered_cup_positions2):
        print(f"Moving to Cup Index {ordered_indices2[index]}: {pose}")
        release()  # 컵을 잡기 전 놓아둠
        pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        x-=10

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
        lowered_position = trans(current_pose, [0, 0, -15.5, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 7️⃣ 컵을 잡기 (grip)
        grip()
        lowered_position = trans(current_pose, [0, 0, 200, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 8️⃣ 컵 배치할 위치로 이동 (높이 보정 후 이동)
        lift_pose = trans(posx(pose), [0, 0, 150, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        lift_pose2 = trans(posx(lift_pose.tolist()), [0, 0, -65, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose2, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 9️⃣ Force 적용 후 컵 배치
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=35):
            rclpy.spin_once(pos_subscriber)

        release_compliance_ctrl()
        release()
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    print("✅ 모든 컵을 배치 완료!")


    ################## 3 층 #####################
        # ✅ 컵을 1개씩 집어서 이동하는 과정
    for index, pose in enumerate(reordered_cup_positions3):
        print(f"Moving to Cup Index {ordered_indices3[index]}: {pose}")
        release()  # 컵을 잡기 전 놓아둠
        #movej(JReady, vel=VELOCITY, acc=ACC)
        pick2=trans(posx(pick_position.tolist()), [0, 0, 100, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick2.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        x-=10

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
        lowered_position = trans(current_pose, [0, 0, 300, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 8️⃣ 컵 배치할 위치로 이동 (높이 보정 후 이동)
        lift_pose = trans(posx(pose), [0, 0, 250, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        lift_pose2 = trans(posx(lift_pose.tolist()), [0, 0, -75, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose2, vel=VELOCITY, acc=ACC, ref=DR_BASE)


        # 9️⃣ Force 적용 후 컵 배치
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=35):
            rclpy.spin_once(pos_subscriber)

        release_compliance_ctrl()
        release()
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        ################## 4 층 #####################
        # ✅ 컵을 1개씩 집어서 이동하는 과정
    for index, pose in enumerate(reordered_cup_positions4):
        print(f"Moving to Cup Index {ordered_indices4[index]}: {pose}")
        release()  # 컵을 잡기 전 놓아둠
        #movej(JReady, vel=VELOCITY, acc=ACC)
        pick2=trans(posx(pick_position.tolist()), [0, 0, 200, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick2.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        pick=trans(posx(pick_position.tolist()), [0, 0, x, 0, 0, 0], DR_BASE, DR_BASE)
        movel(posx(pick.tolist()), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        x-=10

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

        # 6️⃣ 다시 Z축 16mm 하강
        lowered_position = trans(current_pose, [0, 0, -16, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 7️⃣ 컵을 잡기 (grip)
        grip()

        lowered_position = trans(posx(lowered_position.tolist()), [0, 0, 300, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lowered_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 8️⃣ 컵 배치할 위치로 이동 (높이 보정 후 이동)
        lift_pose = trans(posx(pose), [0, 0, 300, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        lift_pose2 = trans(posx(lift_pose.tolist()), [0, 0, -30, 0, 0, 0], DR_BASE, DR_BASE)
        movel(lift_pose2, vel=VELOCITY, acc=ACC, ref=DR_BASE)


        # 9️⃣ Force 적용 후 컵 배치
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=35):
            rclpy.spin_once(pos_subscriber)

        release_compliance_ctrl()
        release()
        movel(lift_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    print("✅ 모든 컵을 배치 완료!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
