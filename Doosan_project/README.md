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

# 1.기어 조립 task

# 동작 영상: https://youtu.be/KN64CLbVrLM

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

## 4️⃣ middle_last() → 최종 기어 배치 및 힘 제어 적용
    함수 middle_last():
        forcing_middle() 실행 → 힘 제어(Force Control) 활성화
        
        반복문 실행:
            Z축 방향의 힘 측정
            만약 힘이 9N 이상 감지되면(=기어 돌리기 전 감지):
                Global_last 위치로 이동(=기어 돌리는 동장)
                기어를 놓음 (release())
                힘 제어 종료 (Force Control 해제)
                반복문 종료 (EXIT)
    
## 메인문
    
    pap(Global_a1, Global_b1)  # 첫 번째 기어 조립
    pap(Global_a2, Global_b2)  # 두 번째 기어 조립
    pap(Global_a3, Global_b3)  # 세 번째 기어 조립
    
    pap_middle()               # 기어 중앙에 새로운 기어 이동
    middle_last()              # 기어 조립

<br><br><br>

---


# 2. 3×3 형태의 팔레트에 블록 길이 순서대로 정렬하는 task
# 동작 영상: https://youtu.be/KN64CLbVrLM

## 1️⃣ 초기 설정 및 기본 동작 정의
    로봇을 기본 위치(JReady)로 이동
    그리퍼(gripper) 관련 잡기(grip), 놓기(release) 동작 함수 정의
    carry() 함수 정의 → 물체를 한 위치에서 다른 위치로 이동하는 동작
    swap() 함수 정의 → 리스트 내 두 개의 원소를 교환하는 함수 정의

## 2️⃣ 3×3 팔레트 좌표 계산
    3×3 배열의 네 꼭짓점 좌표(a1, a2, a3, a4)를 기반으로 좌표 계산
    각 위치의 X, Y 좌표를 균등하게 배치하여 `pallet_positions` 리스트 생성

## 3️⃣ 각 위치의 막대 길이 감지
    모든 `pallet_positions` 위치를 순회하면서:
        1. 해당 위치로 이동
        2. Z축 방향 Force Control 적용
        3. 힘 감지되면 현재 Z 좌표 측정
        4. 감지된 Z 좌표에 따라 막대를 분류:
           - Z > 60 → "긴 막대"
           - 50 < Z ≤ 60 → "중간 막대"
           - Z ≤ 50 → "짧은 막대"
        5. `detected_objects[]` 리스트에 각 위치의 막대 정보 저장
        6. Force Control 해제

## 4️⃣ 감지된 막대 정보를 리스트(arr)에 저장
    각 위치의 `detected_objects` 값을 기반으로:
        - "짧은 막대" → `arr` 리스트에 `1` 추가
        - "중간 막대" → `arr` 리스트에 `2` 추가
        - "긴 막대" → `arr` 리스트에 `3` 추가

## 5️⃣ 긴 막대 하나를 b1(외부 보관 위치)로 이동
    가장 처음 발견한 "긴 막대"를 `b1`로 이동
    이동한 위치를 `empty`로 설정

## 6️⃣ 짧은 막대(1) 정렬 (첫 번째 행)
    첫 번째 행(인덱스 0~2)에 대해:
        - 이미 "짧은 막대(1)"가 있으면 `-1`로 변경 (정렬 완료)
        - 비어 있는 칸(empty)이 있으면, 나머지 영역(3~8)에서 "짧은 막대(1)"를 찾아 이동 후 자리 교환

## 7️⃣ 중간 막대(2) 정렬 (두 번째 행)
    두 번째 행(인덱스 3~5)에 대해:
        - 이미 "중간 막대(2)"가 있으면 `-1`로 변경 (정렬 완료)
        - 비어 있는 칸(empty)이 있으면, 나머지 영역(6~8)에서 "중간 막대(2)"를 찾아 이동 후 자리 교환

## 8️⃣ 긴 막대(3) 정렬 (세 번째 행)
    b1(외부 보관 위치)에 있던 "긴 막대(3)"를 `empty` 위치로 되돌려 정렬 완료

---

<br><br>

# 3. Sport Stacking task
# 동작 영상 https://youtube.com/shorts/2dssJ4UTbxk

## 1️⃣ 초기 설정 및 기본 동작 정의
    로봇의 기본 위치(JReady)로 이동
    그리퍼(gripper) 관련 동작 함수 정의:
        - grip() → 컵을 잡는 함수
        - release() → 컵을 놓는 함수
        - wait_digital_input() → 디지털 입력 신호 대기 함수

## 2️⃣ 컵 좌표 및 이동 경로 설정
    기본 좌표(a0) 설정
    컵이 배치될 10개의 위치(a0 ~ a9) 계산하여 리스트 생성
    컵을 특정 순서로 정렬하기 위해 인덱스 배열(ordered_indices1, ordered_indices2, ordered_indices3) 정의

## 3️⃣ 1층 컵 쌓기 (첫 번째 그룹 - 6개)
    for 각 컵 위치 in reordered_cup_positions1:
        1. `pick_position`에서 컵을 잡음
        2. Force Control 적용 후 컵을 살짝 들어 올림
        3. 컵을 목표 위치로 이동 후 Z축 방향으로 천천히 내림
        4. Force Control 적용 후 컵을 바닥에 맞춰 정렬
        5. 컵을 놓고 상단으로 이동하여 다음 작업을 수행

## 4️⃣ 2층 컵 쌓기 (두 번째 그룹 - 3개)
    for 각 컵 위치 in reordered_cup_positions2:
        1. `pick_position`에서 컵을 잡음
        2. Force Control 적용 후 컵을 살짝 들어 올림
        3. 컵을 목표 위치로 이동 후 Z축 방향으로 천천히 내림
        4. Force Control 적용 후 컵을 바닥에 맞춰 정렬
        5. 컵을 놓고 상단으로 이동하여 다음 작업을 수행

## 5️⃣ 3층 컵 쌓기 (세 번째 그룹 - 1개)
    for 각 컵 위치 in reordered_cup_positions3:
        1. `pick_position`에서 컵을 잡음
        2. Force Control 적용 후 컵을 살짝 들어 올림
        3. 컵을 목표 위치로 이동 후 Z축 방향으로 천천히 내림
        4. Force Control 적용 후 컵을 바닥에 맞춰 정렬
        5. 컵을 놓고 상단으로 이동하여 다음 작업을 수행

## 6️⃣ 4층 컵 추가 작업
    1. 특정 위치에서 컵을 잡음
    2. 다른 위치로 이동하여 컵을 배치

## 7️⃣ 컵을 잡아서 커피 따르기 (Performance 단계)
    1. 특정 위치에서 컵을 잡음
    2. 컵을 커피 따르기 위치로 이동
    3. 컵을 일정한 각도로 기울여 커피 따르기
    4. 물을 따르기 위해 컵을 앞뒤로 회전하며 여러 번 기울임
    5. 커피가 다 따라지면 컵을 원래 위치로 이동

## 8️⃣ 컵을 최종 배치 및 정리
    1. 컵을 특정 위치로 이동하여 배치
    2. 최종 컵 흔들기 동작 수행
    3. 컵을 최종 전달 위치로 이동하여 놓음
    4. 작업 완료 후 초기 위치(JReady)로 복귀


