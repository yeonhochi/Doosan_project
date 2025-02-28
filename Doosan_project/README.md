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
    로봇을 기본 위치(Base Position)로 이동
    grip() 함수 정의 → 그리퍼를 닫아 기어를 잡음
    release() 함수 정의 → 그리퍼를 열어 기어를 놓음
    forcing_middle() 함수 정의 → 힘 제어(Force Control)를 적용하여 정확한 배치 수행

## 2️⃣ pap() → 기어를 잡아서 끼우는 동작 함수
    함수 pap(a, b):
        a 위치에서 150mm 위로 이동
        a 위치로 내려와 기어를 잡음 (grip())
        다시 150mm 위로 이동
        b 위치에서 150mm 위로 이동
        b 위치로 내려와 기어를 놓음 (release())
        다시 150mm 위로 이동하여 대기


## 3️⃣ pap_middle() → 3개의 기어 중앙에 새로운 기어 배치
    함수 pap_middle():
        Global_middle 위치에서 150mm 위로 이동
        Global_middle 위치로 내려와 기어를 잡음 (grip())
        다시 150mm 위로 이동
        Global_last 위치에서 70mm 위로 이동 (최종 기어 배치 위치로 이동)

## 3개의 기어를 조립하는 과정
    pap(Global_a1, Global_b1)  # 첫 번째 기어 조립
    pap(Global_a2, Global_b2)  # 두 번째 기어 조립
    pap(Global_a3, Global_b3)  # 세 번째 기어 조립

    pap_middle()               # 기어 중앙에 새로운 기어 배치

## 4️⃣ middle_last() → 최종 기어 배치 및 힘 제어 적용
    함수 middle_last():
        forcing_middle() 실행 → 힘 제어(Force Control) 활성화
        
        반복문 실행:
            Z축 방향의 힘 측정
            만약 힘이 9N 이상 감지되면:
                Global_last 위치로 이동
                기어를 놓음 (release())
                힘 제어 종료 (Force Control 해제)
                반복문 종료 (EXIT)
    
    middle_last()

## ✅ 기어 조립 완료!
프로그램 종료




