# CLAUDE.md

이 파일은 Claude Code가 이 저장소에서 작업할 때 참고하는 프로젝트 매뉴얼이다.

## 프로젝트 개요

카메라 + 라이다로 주변 장애물을 인지하고, ROS2에서 자율주행 판단을 내려, 아두이노가 그 명령을
CAN 통신으로 후륜구동 모터드라이브에 전달하는 3단계 파이프라인의 자율주행 EV 프로젝트.

```
[카메라 + 라이다] --(/scan, 추후 /camera)--> [ROS2 인지: ev_perception]
                                                    |  /perception/obstacle (ObstacleInfo)
                                                    v
                                          [ROS2 판단: ev_planning]
                                                    |  /vehicle_cmd (VehicleCommand)
                                                    v
                                    [ROS2 브릿지: ev_arduino_bridge] --(USB 시리얼)-->
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
- **`ev_perception`** (ament_python) — `obstacle_detector_node`: `/scan`(LaserScan) 구독, 정면
  각도창(기본 ±60°) 안 최근접 장애물 거리를 `/perception/obstacle`로 발행. 카메라 라바콘 색상
  인식(`~/Desktop/EV_formula_camera` 참고: 오른쪽=파란색, 왼쪽=노란색 HSV)은 **아직 미통합** (TODO).
- **`ev_planning`** (ament_python) — `planning_node`: `/perception/obstacle` 구독, 20Hz로
  `/vehicle_cmd` 발행. 현재 로직은 "정지거리 안에 장애물이면 정지, 아니면 고정 크루즈 스로틀로
  직진"뿐이다. 조향(라바콘 트랙 추종 등)은 항상 0으로 나간다 (TODO).
- **`ev_arduino_bridge`** (ament_python) — `arduino_bridge_node`: `/vehicle_cmd` 구독,
  `/dev/ttyACM0`(기본, 파라미터로 변경 가능)로 `"C,<enable>,<throttle>,<steer>,<brake>\n"` 라인을
  아두이노에 전송.
- **`ev_bringup`** — `launch/ev_stack_launch.py`가 위 세 노드를 묶어 실행. 카메라/라이다 드라이버
  노드(rplidar_ros, usb_cam 등)는 아직 이 launch에 없다 (TODO, 장비 확정 후 추가).

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

# 전체 스택 실행 (카메라/라이다 드라이버는 별도 실행 필요, 검증 완료: 노드 3개 정상 기동)
PATH=/usr/bin:/usr/local/bin:/usr/sbin:/sbin:/bin:$PATH ros2 launch ev_bringup ev_stack_launch.py
```

아두이노 펌웨어는 Arduino IDE(1.8.19, snap 설치됨: `arduino`)로 `firmware/EZkontrol_RearDrive_CAN/
EZkontrol_RearDrive_CAN.ino`를 열어 컴파일/업로드한다. 필요 라이브러리: `due_can`(Collin80,
라이브러리 매니저), 보드 패키지: Arduino SAM Boards, 보드: Arduino Due.

자동화된 테스트는 없다. `ev_perception`/`ev_planning`의 순수 로직(각도 필터링, 정지 판단)은
REPL에서 바로 검증 가능하고, 아두이노/CAN 왕복과 시리얼 브릿지는 실물 하드웨어(EZkontrol
컨트롤러, CAN 트랜시버, Due 보드)가 있어야 검증 가능하다. 단, 아두이노 스케치는 실제 Due 보드
타겟(`arduino:sam:arduino_due_x_dbg`)으로 `--verify` 컴파일까지는 통과했다(2026-08-12).

## 현재 상태 / TODO

- [x] 후륜구동계 CAN 제어 아두이노 펌웨어 (핸드셰이크, 명령 인코딩, 텔레메트리 디코딩, 안전 워치독) — Due 보드 대상 컴파일 검증 완료
- [x] ROS2 인지→판단→아두이노 브릿지 3단계 스캐폴드 (메시지, 노드, launch)
- [x] ROS2 Humble(ros-base) 설치 및 colcon 빌드 검증 (2026-08-12, 5개 패키지 모두 빌드/임포트/launch 성공 — arduino_bridge_node는 하드웨어 미연결로 `/dev/ttyACM0` 없다는 예상된 에러만 남기고 종료)
- [ ] 카메라 라바콘 인식(`~/Desktop/EV_formula_camera`)을 `ev_perception`에 노드로 통합
- [ ] 라이다 드라이버(rplidar_ros 등) 및 카메라 드라이버를 `ev_bringup` launch에 추가
- [ ] `ev_planning`에 실제 트랙 추종/조향 로직 추가 (현재 조향은 항상 0)
- [ ] 조향계/브레이크계(L7SA, Modbus RTU) 제어 인터페이스 — 별도 아두이노 또는 RS485 브릿지 필요
- [ ] 벤치 테스트로 `MAX_TARGET_CURRENT_A` 단계적 상향 및 실차 검증

## 참고 자료

- Golden Motor EZkontrol CAN 프로토콜: `goldenmotor.bike/blogs/ezkontrol-controller/
  ezkontrol-communication-protocols` (MCU-to-VCU, MCU-to-METER, Instruction PDF 3종)
- LS Xmotion L7S 서보드라이브 매뉴얼: `~/Desktop/메뉴얼.pdf`
- 라이다 참고 구현: `~/rplidar_cpp`(SDK), `~/rplidar_front_monitor.py` (RPLidar A3M1,
  `/dev/ttyUSB0`, 256000bps, 전방 120도 감시 — `ev_perception`의 기본 파라미터와 동일 각도 사용)
- 카메라 참고 구현: `~/Desktop/EV_formula_camera` (라바콘 HSV 색상 인식, YOLO 미사용)
