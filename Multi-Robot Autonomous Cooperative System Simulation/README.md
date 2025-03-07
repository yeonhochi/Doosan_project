

## 📌 프로젝트 개요
**PatrolX**는 교도소 내 보안을 강화하기 위해 개발된 **지능형 순찰 로봇 시스템**입니다.  
2대의 로봇이 협력하여 방 내 인원을 감지하고, 탈옥범을 탐지하여 자동으로 대응할 수 있도록 설계되었습니다.

<img src="https://github.com/user-attachments/assets/b473fd9f-5a53-4bf8-b57a-7a8f4404dc2c" width="500">

---

## 시스템 구성
- **순찰 로봇 2대**: 각각 1~6번 방, 7~12번 방을 담당
- **감지 기능**: 방 내 사람 유무 감지 및 탈옥범 감지
- **협업 기능**: 비정상 감지 시 로봇 간 이동 및 협력 수행
- **핵심 개념**: 로봇을 `tb1`, `tb2` 두 개의 네임스페이스로 나누어 운영

---

## 시스템 동작 방식

1️**순찰 시작**  
   - 각 로봇이 지정된 구역의 방을 순찰  
2️**비정상 감지**  
   - 사람이 없어야 할 방에 사람이 있거나, 탈옥범이 감지되면 관리자에게 알림 전송  
3️**로봇 이동**  
   - 관리자 명령을 받아 해당 위치로 긴급 출동 후 탐색 수행  
4️**탈옥범 발견**  
   - 발견한 로봇은 정지하고, 다른 로봇이 협력하여 탈옥자를 제압  

---

## 맵 제작
- **Blender**를 이용한 실내 맵 제작  

---

## GUI (관리자 시스템)
- **AMR에서 받은 이미지**를 압축 후 GUI로 전송하여 실시간 감시 가능
- 
<img src="https://github.com/user-attachments/assets/4ca174a6-7777-4dda-a437-16ce33d8a391" width="400">

---

## 시스템 구현

### Waypoint 기반 순찰
- 각 로봇은 **순찰 구역을 Waypoint로 설정**하여 반복 순찰 수행

### 긴급 상황 대응 (`/emergency`)
- `/emergency` 토픽 수신 시 `stop_navigation` 실행 → 순찰 중단  
- `navigate_to_emergency_location` 실행 → 지정된 긴급 위치로 이동 후 탐색 수행  

### 탈옥범 감지 (`/found`)
- 탈옥범 발견 시 `/found` 토픽 발행 → 해당 로봇은 정지하고 위치 정보 전송  
- 이를 수신한 다른 로봇이 해당 위치로 이동하여 협력 대응



<img src="https://github.com/user-attachments/assets/e71f48dd-982f-4afd-bc0a-2b72f2b28ce3" width="500">
<img src="https://github.com/user-attachments/assets/6836f38e-2e6c-44e3-9466-802c28ba749c" width="500">


---

## 알림 시스템
- **탈옥 발생** → 관리자 출동 버튼 클릭 시 `emergency.mp3` 재생  
- **탈옥범 발견** → 감지 시 `found.mp3` 재생  

---

## 동작 영상
🔗 [기본 탐색 및 추적 모드 영상](https://gamma.app/?utm_source=made-with-gamma)  

---

