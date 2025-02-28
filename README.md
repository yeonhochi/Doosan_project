<img src="https://github.com/user-attachments/assets/bc8154a5-77aa-4bbb-8796-d6f29ee5c2d6" width="300">

# DOOSAN 협동로봇 프로젝트 개요

DOOSAN 로봇 프로젝트는 **두산로보틱스 M0609 모델**을 사용하며, `DSR`(Doosan Software for Robotics) 패키지와 `ROS`(Robot Operating System) 환경을 활용하여 다양한 task를 진행합니다.  

---

## 📌 프로젝트 주요 Task

이 프로젝트는 다음과 같은 task들로 구성됩니다:
1. **기어 조립 task**
2. **3×3 형태의 팔레트에 블록 길이 순서대로 배치하는 task**
3. **Sport Stacking task**


ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609


ros2 run rokey jog

ros2 run rokey get_current_pos

ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "robot_mode: 0"

---

1. 로봇 시스템 초기화
    - ROS2 초기화
    - 현재 위치 구독 노드 생성
    - 로봇의 기본 설정 (Tool, TCP 설정)

2. 기본 준비 위치 (JReady)로 이동

3. 컵 배치할 좌표 계산
    - 컵 개수 설정 (총 14개)
    - a1 (시작점) → a2 (끝점) 사이의 보간값으로 컵 위치 생성
    - 배치 순서 조정 (1층, 2층, 3층)

4. **픽업 위치 변수 초기화**
    - `x = 0`  (픽업 높이 조정 변수)

5. **1층 컵 배치**
    - for each `pose` in `reordered_cup_positions1`:
        1. 컵을 잡기 전 그리퍼 열기 (release)
        2. **픽업 위치 보정 (`x -= 10`)**
        3. 컵 픽업 위치로 이동 (`pick_position` + `x` 보정)
        4. 컵을 잡기 (grip)
        5. Force Control 적용 후 확인
        6. Z축으로 상승 (13mm)
        7. 컵 배치할 위치로 이동
        8. Force Control 적용 후 컵 배치
        9. Z축으로 다시 상승 (이탈)
    - **1층 완료 메시지 출력**

6. **2층 컵 배치** (1층과 동일한 과정, 다만 높이 조절 필요)
    - for each `pose` in `reordered_cup_positions2`:
        1. 컵을 잡기 전 그리퍼 열기 (release)
        2. **픽업 위치 보정 (`x -= 10`)**
        3. 컵 픽업 위치로 이동 (`pick_position` + `x` 보정)
        4. 컵을 잡기 (grip)
        5. Force Control 적용 후 확인
        6. Z축으로 상승 (13mm)
        7. 컵 배치할 위치로 이동 (높이 150mm 보정)
        8. Force Control 적용 후 컵 배치
        9. Z축으로 다시 상승 (이탈)
    - **2층 완료 메시지 출력**

7. **3층 컵 배치** (높이 250mm 보정 필요)
    - for each `pose` in `reordered_cup_positions3`:
        1. 컵을 잡기 전 그리퍼 열기 (release)
        2. **픽업 위치 보정 (`x -= 10`)**
        3. 컵 픽업 위치로 이동 (`pick_position` + `x` 보정)
        4. 컵을 잡기 (grip)
        5. Force Control 적용 후 확인
        6. Z축으로 상승 (13mm)
        7. 컵 배치할 위치로 이동 (높이 250mm 보정)
        8. Force Control 적용 후 컵 배치
        9. Z축으로 다시 상승 (이탈)
    - **3층 완료 메시지 출력**

8. 모든 컵 배치 완료 후 시스템 종료 (`rclpy.shutdown()`)
