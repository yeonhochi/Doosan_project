<img src="https://github.com/user-attachments/assets/bc8154a5-77aa-4bbb-8796-d6f29ee5c2d6" width="300">

# DOOSAN 협동로봇 프로젝트 개요

DOOSAN 로봇 프로젝트는 **두산로보틱스 M0609 모델**을 사용하며, `DSR`(Doosan Software for Robotics) 패키지와 `ROS`(Robot Operating System) 환경을 활용하여 다양한 task를 진행합니다.  

---

## 📌 프로젝트 주요 Task

이 프로젝트는 다음과 같은 task들로 구성됩니다:
1. **기어 조립 task**
2. **3×3 형태의 팔레트에 블록 길이 순서대로 배치하는 task**
3. **Sport Stacking task**

---
<br>

# 동작 영상 https://youtube.com/shorts/2dssJ4UTbxk

<br>

---

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609


ros2 run rokey jog

ros2 run rokey get_current_pos

ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "robot_mode: 0"

---

# 기어 조립 코드 설명

## 1️⃣ 초기 설정 및 기본 동작 정의

    로봇이 기본 위치(base position)로 이동
    그리퍼(gripper) 관련 잡기(grip), 놓기(release) 동작 함수 정의
    힘 제어(force control)를 위한 forcing_middle() 함수 정의

## 2️⃣ pap() → 기어를 잡아서 끼우는 동작

✅ 기능:

    기어를 들어서 목적지에 배치하는 동작
    a 위치에서 기어를 집고 b 위치에 기어를 놓음

✅ 동작 순서:

    a 위치 위쪽(150mm)으로 이동
    a 위치로 내려와 기어를 잡음 (grip())
    다시 위쪽(150mm)으로 올라감
    b 위치 위쪽(150mm)으로 이동
    b 위치로 내려와 기어를 놓음 (release())
    다시 위쪽(150mm)으로 올라가 대기

📌 이 함수는 Global_a1 → Global_b1, Global_a2 → Global_b2, Global_a3 → Global_b3 로 기어를 조립
## 3️⃣ pap_middle() → 3개의 기어 중앙에 새로운 기어 배치

✅ 기능:

    이미 조립된 3개의 기어 중심에 추가 기어를 올리는 동작

✅ 동작 순서:

    Global_middle 위치 위쪽(150mm)으로 이동
    Global_middle 위치로 내려와 기어를 잡음 (grip())
    다시 위쪽(150mm)으로 올라감
    Global_last 위치 위쪽(70mm)으로 이동 (최종 기어 위치)

📌 이 동작은 기존 기어 위에 새로운 기어를 올리는 과정

## 4️⃣ middle_last() → 최종 기어 배치 및 힘 제어 적용

✅ 기능:

    최종 기어를 놓을 때 힘(force)을 감지하여 안정적으로 배치
    힘이 일정 이상 감지되면 기어를 놓고(force release) 힘 제어를 해제

✅ 동작 순서:

    forcing_middle() 함수를 실행하여 힘 제어(Force Control) 시작
    반복문을 돌며 Z축 힘을 측정
    특정 힘 이상(get_tool_force(DR_BASE)[2] > 9) 감지되면
        Global_last 위치로 이동
        기어를 놓음 (release())
        힘 제어를 중지하고 종료

📌 이 동작은 마지막 기어를 기존 기어들과 맞물리도록 배치하는 과정



