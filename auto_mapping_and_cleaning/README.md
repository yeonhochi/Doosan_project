<img src="https://github.com/user-attachments/assets/a66d203d-aad3-4c32-935c-236dfc5f3d3f" width="500">

<img src="https://github.com/user-attachments/assets/3916763b-f4cd-44e2-84ce-ea3dce010875" width="500" height="550">


# auto_mapping_cleaning 프로젝트 개요

`auto_mapping_cleaning` 프로젝트는 **ROS2 (Robot Operating System)** 환경에서 **자동 매핑 및 청소 경로 생성**을 목표로 합니다.  
이 시스템은 **프론티어 탐색 알고리즘**을 활용하여 **미탐지 구역을 효과적으로 탐색**하고, 생성된 맵을 기반으로 **최적의 청소 경로를 자동으로 계획**합니다.

---

## 📌 프로젝트 주요 Task

이 프로젝트는 다음과 같은 주요 기능을 포함합니다:

1. **프론티어 알고리즘 기반 미탐지 구역 탐색**
   - LiDAR 센서를 활용한 실시간 환경 스캔  
   - 탐색이 완료되지 않은 영역을 자동으로 감지 및 이동  

2. **생성된 맵 기반 청소 경로 생성**
   - SLAM을 통해 구축된 맵을 활용한 경로 계획  
 
---

# mapping 코드 설명

1️⃣ 초기화 및 구독/발행 설정

    /map 토픽을 구독하여 OccupancyGrid(맵 데이터) 를 가져옴
    /follow_path/_action/status 토픽을 구독하여 로봇의 목표 도착 여부를 확인
    /odom을 구독하여 로봇의 현재 위치(x, y) 정보를 업데이트
    goal_pose 토픽을 발행하여 다음 목표 지점(goal)을 전송

2️⃣ map_callback() → 맵을 numpy 배열로 변환

    ROS 2의 OccupancyGrid 메시지를 numpy 배열로 변환하여 맵 데이터로 저장
    맵의 크기(너비, 높이) 및 해상도(metadata)도 함께 저장

3️⃣ goal_status_callback() → 목표 도착 여부 확인

    로봇이 목표 위치에 도착했는지 확인
        도착했으면 goal_reached = True
        실패(Aborted)한 경우에도 새로운 목표를 설정할 수 있도록 goal_reached = True로 변경

4️⃣ timer_callback() → 5초마다 새로운 목표 선정

    맵을 아직 받지 못했거나(self.map_array is None)
    이미 목표 지점으로 이동 중이라면(goal_reached = False) → 새로운 목표를 설정하지 않음
    미탐색 지역(Unknown: -1)과 인접한 이동 가능 영역(Free: 0)을 찾음 (프론티어 탐색)
    장애물과 너무 가까운 프론티어는 제외
    후보 중 하나를 랜덤으로 선택하여 목표 지점으로 설정

5️⃣ detect_frontiers() → 미탐색 영역 찾기

    맵에서 값이 -1인 셀(미탐색 영역) 중, 주변에 0(이동 가능 공간)이 있는 경우를 탐색 경계(frontier)로 분류
    즉, 로봇이 접근하면 탐색이 확장될 수 있는 영역을 찾아 리스트로 반환

6️⃣ is_near_wall() → 장애물(벽)과 가까운지 확인

    후보 지점이 장애물(값=100)과 일정 거리 이내라면 탐색 대상으로 제외
    이 과정을 통해 로봇이 벽에 부딪히는 것을 방지

7️⃣ select_goal() → 목표 좌표를 선택하여 실제 좌표 변환

    랜덤으로 프론티어 후보 중 하나를 선택
    맵의 픽셀 좌표(col, row) → 실제 월드 좌표(x, y) 로 변환
    변환식:

    real_x = col * resolution + origin_x
    real_y = row * resolution + origin_y

8️⃣ publish_goal() → 목표 좌표를 발행하여 로봇 이동

    PoseStamped 메시지 생성하여 goal_pose 토픽으로 발행
    로봇은 Nav2 경로 계획을 통해 해당 지점으로 이동
    목표가 설정되면 goal_reached = False로 변경하여 중복 목표를 방지

9️⃣ main() → ROS 2 노드 실행

    ROS 2 노드를 실행하고 탐색을 지속적으로 수행


