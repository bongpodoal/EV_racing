# CLAUDE.md

이 파일은 Claude Code가 이 저장소에서 작업할 때 참고하는 프로젝트 매뉴얼이다.

## 프로젝트 개요

카메라 + 라이다로 주변 장애물을 인지하고, ROS2에서 자율주행 판단을 내려, 아두이노가 그 명령을
CAN 통신으로 후륜구동 모터드라이브에 전달하는 3단계 파이프라인의 자율주행 EV 프로젝트.

```
[RPLidar A3M1]  --/scan-->       [obstacle_detector_node] --/perception/obstacle-->\
                                                                                      [planning_node] --/vehicle_cmd--> [arduino_bridge_node]
[v4l2 카메라]   --/image_raw-->  [cone_detector_node]      --/perception/cones---->/        |
                                                                                       (USB 시리얼)
                                                                                              v
                                          [Arduino Due 펌웨어] --(CAN)--> [EZkontrol B481000] --> [HPM10KW 모터]
```

**주의**: 홈 디렉토리의 `~/github/henes`(GPS RTK + Pure Pursuit 웨이포인트 주행, H-브리지 모터,
SBUS RC, Arduino Mega)는 **다른 차량의 별개 프로젝트**다. 이 저장소와 코드/의존성을 공유하지 않는다.

## 하드웨어 구성

### 후륜구동계

| 항목 | 값 |
|---|---|
| 모터 | Golden Motor HPM10KW (48V / 10KW, 공랭식) |
| 컨트롤러 | EZkontrol B481000 (정격 400A / 최대 1000A, 6~10KW 모터 매칭) |
| 통신 | CAN, 컨트롤러 CN2 커넥터: CN2-10(황색)=CAN_H, CN2-21(녹색)=CAN_L, CN2-11(갈색)=CAN_TERM(120Ω) |

### 조향계 / 브레이크계 (현재 이 저장소의 소프트웨어 범위 밖 — TODO)

| 항목 | 조향계 | 브레이크계 |
|---|---|---|
| 서보모터 | LS Xmotion APMC-FBL04AMK (400W, 3상 220V, 2.60A, 3000rpm, IP67) | LS Xmotion 200W급 |
| 드라이브 | L7SA004B (정격 3.0A / 최대 9.0A) | L7SA002B (정격 1.7A / 최대 5.1A) |
| 통신 | Modbus RTU (RS485) — CAN 아님 | Modbus RTU (RS485) — CAN 아님 |

드라이브 매뉴얼: `~/Desktop/메뉴얼.pdf` (LS Xmotion L7S Series 사용설명서 Ver1.6, Modbus 레지스터
Function Code 0x03/0x06 구조 포함). 조향/브레이크 제어 노드·아두이노 인터페이스는 아직 없다.

## 소프트웨어 아키텍처

### ROS2 워크스페이스 (`ros2_ws/src/`)

- **`ev_interfaces`** (ament_cmake) — 커스텀 메시지
  - `ObstacleInfo.msg`: `header`, `obstacle_detected`, `distance_m`, `angle_rad`
  - `VehicleCommand.msg`: `enable`, `throttle_percent`, `steering_deg`, `brake_percent`
  - `Cone.msg`: `color`("YELLOW"/"BLUE"), `side`("LEFT"/"RIGHT"/"CENTER"), `side_ok`, `cx`, `cy`, `area`
  - `ConeArray.msg`: `header`, `image_width`, `Cone[] cones`
- **`ev_perception`** (ament_python)
  - `obstacle_detector_node`: `/scan`(LaserScan) 구독, 정면 각도창(기본 ±60°) 안 최근접 장애물
    거리를 `/perception/obstacle`로 발행.
  - `cone_detector_node`: `/image_raw`(sensor_msgs/Image, v4l2_camera_node 기본 토픽) 구독,
    `cv_bridge`로 OpenCV 프레임 변환 후 `cone_detection.detect_cones()`로 라바콘(YELLOW=왼쪽 기대,
    BLUE=오른쪽 기대)을 검출해 `/perception/cones`로 발행. `cv_bridge` 변환 실패(카메라가 bgr8로
    못 바꾸는 인코딩을 보내는 경우 등)는 그 프레임만 버리고 노드는 계속 살아있는다(try/except).
  - `cone_detection.py`: `~/Desktop/EV_formula_camera`에서 그대로 포팅한 순수 HSV 검출 로직
    (`build_masks`/`find_cones`/`classify_side`/`detect_cones`) — 카메라 캡처는 이 파일에 없고
    ROS2 Image 구독(`cone_detector_node`)이 대신한다. HSV 임계값·면적·종횡비 기준은 원본 그대로.
- **`ev_planning`** (ament_python) — `planning_node`: `/perception/obstacle` + `/perception/cones`
  구독, 20Hz로 `/vehicle_cmd` 발행. 우선순위: **라이다 장애물 정지 > 콘 트랙 추종**.
  - 트랙 추종: `side_ok=True`인(색상-위치 불일치 없는) 콘만 대상으로, 면적이 가장 큰(=가장 가까운)
    YELLOW/BLUE 쌍의 이미지 x좌표 중점으로 조향각 계산(`_compute_steering_deg`). 한쪽만 보이면
    `assumed_track_half_width_px`만큼 반대쪽에 게이트가 있다고 가정. 둘 다 안 보이면(코너 진입 등)
    `search_throttle_percent`로 감속 직진.
  - 인지 자체가 끊기면(라이다든 카메라든 `perception_timeout_s` 초과) 무조건 정지 — 안전 기본값.
  - `max_steer_deg` 기본 25°는 실측값이 아니라 `henes_control.ino`의 실측 최대 조향각(±25°)을
    임시로 참고한 자리표시값이다 (TODO: EV_racing 실차로 교체).
- **`ev_arduino_bridge`** (ament_python) — `arduino_bridge_node`: `/vehicle_cmd` 구독,
  `/dev/ttyACM0`(기본, 파라미터로 변경 가능)로 `"C,<enable>,<throttle>,<steer>,<brake>\n"` 라인을
  아두이노에 전송. `steer_deg`/`brake_percent`는 계산은 되지만 아두이노 펌웨어가 아직 안 씀(TODO).
- **`ev_bringup`** — `launch/ev_stack_launch.py`가 드라이버 2개(`rplidar_ros`의 `rplidar_node`,
  `v4l2_camera`의 `v4l2_camera_node`) + 앱 노드 4개를 묶어 실행. 파라미터는 실측 확인값이 기본값:
  라이다 `/dev/ttyUSB0`/256000bps(A3M1), 카메라 `/dev/video0`.
  **알려진 문제**: `ros-humble-rplidar-ros`(2.1.4) 패키지는 지정한 시리얼 포트가 존재하지 않으면
  깔끔한 에러 대신 `*** buffer overflow detected ***`로 강제 종료된다(2026-08-12 재현 확인,
  하드웨어 미연결 상태). launch 시스템은 이 크래시와 무관하게 나머지 노드는 정상 유지하지만,
  이 노드 자체의 정상 동작 검증은 실제 A3M1을 연결해야만 가능하다 — 업스트림 버그로 보이며 이
  저장소 코드의 문제는 아님.

### 아두이노 펌웨어 (`firmware/EZkontrol_RearDrive_CAN/`)

Arduino Due + `due_can` 라이브러리. Golden Motor "EZkontrol MCU to VCU CAN Protocols V1.0
20221001" 공식 문서(goldenmotor.bike)를 그대로 구현:

- 250Kbps, 29비트 확장 CAN ID. 명령(VCU→MCU) `0x0C01EFD0`, 텔레메트리(MCU→VCU)
  `0x1801D0EF`(전압/전류/속도), `0x1802D0EF`(온도/상태/에러).
- 핸드셰이크: MCU가 0x55×8을 보내는 동안 VCU가 0xAA×8로 응답 → 이후 50ms 주기로 실제 명령/텔레메트리
  교환. 텔레메트리 500ms 끊기면 재핸드셰이크 상태로 복귀.
- 목표 전류 0.1A/bit(offset -3200A), 목표 속도 1rpm/bit(offset -32000rpm)로 인코딩.
- **시리얼 입력 두 가지 모드**: `ev_arduino_bridge`가 보내는 `"C,..."` 라인이 최근(300ms 이내)
  수신되면 그 값을 쓰고, 없으면 `A0` 아날로그 스로틀로 폴백해 ROS2 없이도 단독 벤치 테스트 가능.
  `steer_deg`/`brake_percent`는 이 펌웨어에서 아직 쓰지 않는다.
- 안전장치: `ENABLE_PIN`(22번, INPUT_PULLUP, 미연결 시 기본 정지) 하드웨어 인터록 + 시리얼/CAN
  타임아웃 워치독. `MAX_TARGET_CURRENT_A`가 안전상 낮은 값(10A)으로 걸려 있음 — 정격 400A/최대
  1000A 컨트롤러이므로 벤치(바퀴 지면에서 뜬 상태) 검증 후에만 단계적으로 상향할 것.
- CAN 프로토콜 원본 PDF는 프로젝트 밖(`/tmp` 스크래치 경로, goldenmotor.bike에서 재다운로드
  가능)에 있고 이 저장소에는 커밋하지 않았다.

## Commands

ROS2 Humble(`ros-base` + `ros-dev-tools`, RViz/Gazebo 없는 경량 설치 — 디스크 여유가 없어 desktop
대신 이걸 선택함)이 설치되어 있다. **주의**: 이 머신은 `~/.bashrc`에서 miniforge(conda)를 자동
활성화하는데, conda의 python3(3.13)가 PATH 앞쪽을 차지해 ROS2용 시스템 python3(3.10, `em`/
`serial` 모듈이 여기 있음)를 가린다. 그래서 ROS2 관련 명령은 항상 `/usr/bin`을 PATH 앞에 두고
실행해야 한다 (안 그러면 `colcon build`가 `No module named 'em'`으로 실패함):

```bash
source /opt/ros/humble/setup.bash

# 워크스페이스 빌드 (검증 완료, 2026-08-12: 5개 패키지 정상 빌드)
cd ros2_ws
PATH=/usr/bin:/usr/local/bin:/usr/sbin:/sbin:/bin:$PATH colcon build --symlink-install
source install/setup.bash

# 전체 스택 실행 (드라이버 2개 + 앱 노드 4개, 검증 완료: rplidar_node 제외 5개 정상 기동 — 위 "알려진 문제" 참고)
PATH=/usr/bin:/usr/local/bin:/usr/sbin:/sbin:/bin:$PATH ros2 launch ev_bringup ev_stack_launch.py
```

카메라/라이다 드라이버 + 콘 인식에 필요한 apt 패키지(2026-08-12 설치 완료):
`ros-humble-cv-bridge`, `ros-humble-rplidar-ros`, `ros-humble-v4l2-camera`, `python3-opencv`.

아두이노 펌웨어는 Arduino IDE(1.8.19, snap 설치됨: `arduino`)로 `firmware/EZkontrol_RearDrive_CAN/
EZkontrol_RearDrive_CAN.ino`를 열어 컴파일/업로드한다. 필요 라이브러리: `due_can`(Collin80,
라이브러리 매니저), 보드 패키지: Arduino SAM Boards, 보드: Arduino Due.

`ev_perception`(콘 검출)과 `ev_planning`(조향 계산)에는 pytest 유닛 테스트가 있다(13개, 전부
합성 데이터로 실카메라/실하드웨어 불필요):

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PATH=/usr/bin:/usr/local/bin:/usr/sbin:/sbin:/bin:$PATH /usr/bin/python3 -m pytest \
  ros2_ws/src/ev_perception/test ros2_ws/src/ev_planning/test -v
```

그 외 자동화된 테스트는 없다. 아두이노/CAN 왕복과 시리얼 브릿지는 실물 하드웨어(EZkontrol
컨트롤러, CAN 트랜시버, Due 보드)가 있어야 검증 가능하다. 단, 아두이노 스케치는 실제 Due 보드
타겟(`arduino:sam:arduino_due_x_dbg`)으로 `--verify` 컴파일까지는 통과했다(2026-08-12).

## 현재 상태 / TODO

- [x] 후륜구동계 CAN 제어 아두이노 펌웨어 (핸드셰이크, 명령 인코딩, 텔레메트리 디코딩, 안전 워치독) — Due 보드 대상 컴파일 검증 완료
- [x] ROS2 인지→판단→아두이노 브릿지 파이프라인 스캐폴드 (메시지, 노드, launch)
- [x] ROS2 Humble(ros-base) 설치 및 colcon 빌드 검증
- [x] 카메라 라바콘 인식(`~/Desktop/EV_formula_camera`)을 `ev_perception`의 `cone_detector_node`로 통합 — pytest 유닛 테스트 6개로 검증(2026-08-12), 실카메라/실라바콘으로는 아직 미검증
- [x] 라이다(`rplidar_ros`) 및 카메라(`v4l2_camera`) 드라이버를 `ev_bringup` launch에 추가 — rplidar_node는 위 "알려진 문제" 있음
- [x] `ev_planning`에 콘 게이트 중심 추종 조향 로직 추가 (`_compute_steering_deg`, pytest 유닛 테스트 7개: 양쪽/한쪽만/미검출/side_ok 필터/최대면적 선택/클램핑)
- [x] `/code-review` 전체 리뷰(2026-08-12) 반영: (1) `_compute_steering_deg`가 `side_ok=False`(오검출 의심) 콘을 걸러내지 않던 버그 수정, (2) `cone_detector_node`의 `cv_bridge` 변환 실패가 노드를 죽이던 문제를 try/except로 방어, (3) "유닛 테스트로 검증" 주장을 실제 커밋된 pytest 파일로 뒷받침. 나머지 2개 지적(launch 인자 선언 패턴, `max()` 재계산)은 각각 `rplidar_ros` 공식 launch와 동일 관용구/의도적 방어 코드로 판단해 반영 안 함.
- [ ] 실제 라바콘 트랙에서 HSV 임계값·`assumed_track_half_width_px`·`steer_gain_deg`·`max_steer_deg` 캘리브레이션 (전부 임의값/추정값)
- [ ] 조향계/브레이크계(L7SA, Modbus RTU) 제어 인터페이스 — 별도 아두이노 또는 RS485 브릿지 필요, `VehicleCommand.steering_deg`/`brake_percent`는 계산되지만 아직 어떤 하드웨어도 소비하지 않음
- [ ] 벤치 테스트로 `MAX_TARGET_CURRENT_A` 단계적 상향 및 실차 검증
- [ ] `rplidar_ros` buffer overflow가 실제 A3M1 연결 시에도 재현되는지 확인 (지금은 장치 없을 때만 확인함)

## 참고 자료

- Golden Motor EZkontrol CAN 프로토콜: `goldenmotor.bike/blogs/ezkontrol-controller/
  ezkontrol-communication-protocols` (MCU-to-VCU, MCU-to-METER, Instruction PDF 3종)
- LS Xmotion L7S 서보드라이브 매뉴얼: `~/Desktop/메뉴얼.pdf`
- 라이다 참고 구현: `~/rplidar_cpp`(SDK), `~/rplidar_front_monitor.py` (RPLidar A3M1,
  `/dev/ttyUSB0`, 256000bps, 전방 120도 감시 — `ev_perception`의 기본 파라미터와 동일 각도 사용)
- 카메라 참고 구현: `~/Desktop/EV_formula_camera` (라바콘 HSV 색상 인식, YOLO 미사용)
