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

https://github.com/yeonhochi/Doosan_project/tree/main/Doosan_project
