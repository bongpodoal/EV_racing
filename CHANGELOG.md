# CHANGELOG

프로젝트 진행 기록. 커밋 메시지보다 상위 레벨에서 "그날 무엇이 왜 만들어졌는지"를 남긴다.
기술적 세부사항(아키텍처, 메시지 스펙, 알려진 문제 등)은 항상 [CLAUDE.md](./CLAUDE.md)가 최신본이다.

## 2026-08-12

**후륜구동계 CAN 제어 아두이노 펌웨어** (`0f1e8ee`)
Golden Motor EZkontrol B481000 컨트롤러의 공식 CAN 프로토콜 문서(goldenmotor.bike)를 확보해
Arduino Due + `due_can` 라이브러리로 구현. 핸드셰이크, 목표 전류/속도 인코딩, 텔레메트리 디코딩,
시리얼(USB)로 ROS2 명령을 받되 연결이 끊기면 아날로그 스로틀로 폴백하는 이중 모드, 하드웨어
인터록 + 통신 타임아웃 워치독까지 포함. 실제 Due 보드 대상 컴파일 검증 완료.

**ROS2 인지→판단→제어 파이프라인 스캐폴드** (`0f1e8ee`, `b46242c`)
`ev_interfaces`(메시지) / `ev_perception`(라이다 장애물 인지) / `ev_planning`(판단) /
`ev_arduino_bridge`(시리얼 브릿지) / `ev_bringup`(launch) 5개 패키지 구성. ROS2 Humble
(`ros-base`, 디스크 여유가 없어 `desktop` 대신 경량 설치)을 설치하고 실제로 `colcon build` +
`ros2 launch`까지 검증. 이 과정에서 이 머신 특유의 conda(miniforge)가 시스템 python3를 가리는
문제를 발견해 해결.

**카메라 콘 인식 + 라이다/카메라 드라이버 + 트랙 추종** (`88f49f3`)
`~/Desktop/EV_formula_camera`의 HSV 라바콘 검출 로직(노랑=왼쪽, 파랑=오른쪽)을 `ev_perception`에
포팅. `rplidar_ros`(A3M1)와 `v4l2_camera` 드라이버를 `ev_bringup` launch에 연결. `ev_planning`에
가장 가까운 좌/우 콘 쌍의 중점으로 조향하는 게이트 중심 추종 로직 추가, 라이다 장애물 정지가
항상 우선하도록 구성. 이 과정에서 `ros-humble-rplidar-ros` 2.1.4의 업스트림 버그(시리얼 포트가
없을 때 깔끔한 에러 대신 buffer overflow로 강제종료)를 발견해 기록.

**`/code-review high` 전체 리뷰 및 수정** (`70d8fd3`)
저장소 전체(펌웨어 + ROS2 패키지 5개)를 코드 리뷰. 실제 버그 2개 수정: (1) 조향 계산이
`side_ok=False`(색상-위치 불일치, 오검출 의심) 콘을 걸러내지 않아 잘못된 방향으로 조향할 수
있던 문제, (2) 카메라 이미지 변환 실패 시 노드 전체가 죽던 문제(`cv_bridge` 예외 미처리).
"유닛 테스트로 검증"이라고만 적혀 있고 실제 테스트 파일이 없던 것도 지적받아 pytest 13개를
실제로 작성/커밋(전부 통과, 하드웨어 불필요). 나머지 2개 지적(launch 인자 선언 패턴, 정렬된
리스트에서 `max()` 재계산)은 각각 업스트림 관용구/의도적 방어 코드로 판단해 반영하지 않음.
