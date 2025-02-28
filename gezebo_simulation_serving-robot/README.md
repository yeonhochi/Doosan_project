<img src="https://github.com/user-attachments/assets/4eee35e9-ee99-4929-a51e-ab6be293dad3" width="300">
<img src="https://github.com/user-attachments/assets/8b217431-a83e-4c19-9f4b-43c5e3939e38" width="300">
<img src="https://github.com/user-attachments/assets/6c5b0478-b91b-4f07-8806-5fd1364cb4d2" width="300">



# 🍽️ ROS 2 기반 서빙 로봇 프로젝트

## 📌 프로젝트 개요
- **목적**: 음식점 서빙 자동화를 통해 효율성 및 정확성 향상  
- **기대효과**:
  - 인건비 절감 및 생산성 향상
  - 주문 정확성 및 신속한 서비스 제공
  - 데이터 기반 메뉴 및 매출 최적화

---

## 🎯 주요 목표
1. **서빙 자동화**: ROS 2 기반 서빙 로봇 개발 (`Navigation2` 활용)  
2. **주문 관리**: 주문을 실시간으로 주방에 전달 및 DB 저장  
3. **데이터 분석**: 매출 통계 및 품절 관리  

---

## 🛠️ 주요 기능
### 🔹 1. 테이블 오더 시스템  
- 고객 주문을 **ROS 2 토픽**을 통해 주방으로 전달  
- 메시지 형식: `table_id, menu_item, quantity`

### 🔹 2. 주방 디스플레이  
- 실시간 주문 확인 및 서빙 명령 발행  
- **GUI** 기반 주문 관리 (PySide2 사용)

### 🔹 3. 서빙 로봇  
- **ROS 2 액션 (`NavigateToPose`)** 활용하여 테이블 이동  
- 장애물 극복을 위한 복구 동작 (Spin, Backup, Wait)

### 🔹 4. 데이터베이스 연동  
- 주문 데이터 저장 및 **실시간 품절 관리**  
- 매출 통계를 저장하고 분석 제공  

### 🔹 5. 통계 및 분석  
- **일일 매출, 인기 메뉴, 선호도 분석** 등 시각화 제공  

---

## 🏗️ 기술 스택
- **ROS 2**: `Navigation2`, QoS 설정, 메시지 인터페이스  
- **Python**: 노드 구현, GUI 연동  
- **DB 관리**: 주문 데이터 및 품절 항목 저장  

---
