/*
  ============================================================================
  후륜구동계 CAN 제어  (Arduino Due  ->  EZkontrol B481000  ->  Golden Motor HPM10KW)
  ============================================================================

  출처: Golden Motor "EZkontrol MCU to VCU CAN Protocols V1.0 20221001" 공식 문서
        (goldenmotor.bike 에서 다운로드한 원문 기준으로 CAN ID/데이터 포맷을 그대로 반영함)

  하드웨어 연결
  -------------
  - Arduino Due는 CN2 커넥터와 직접 연결되지 않습니다. Due의 CAN0(또는 CAN1) 핀에
    외부 CAN 트랜시버(예: MCP2551, SN65HVD230/TJA1051 등, 3.3V 신호 레벨 확인)를 연결하고,
    트랜시버의 CANH/CANL을 컨트롤러 CN2 커넥터에 배선합니다.
      CN2-10 (황색선) = CAN_H
      CN2-21 (녹색선) = CAN_L
      CN2-11 (갈색선) = CAN_TERM (120Ω 종단저항, 출고 기본 연결됨)
  - Due 보드 실물에서 CAN0/CAN1의 정확한 핀 위치는 보드 핀맵(due_can 라이브러리 예제 참고)으로
    반드시 확인 후 배선하세요. 버스 양 끝단에만 120Ω 종단저항이 있어야 합니다(컨트롤러 쪽은 이미 있음).

  필요 라이브러리
  -------------
  - "due_can" (Collin80) : Arduino IDE > 스케치 > 라이브러리 포함하기 > 라이브러리 관리에서 설치
  - 보드 패키지 "Arduino SAM Boards (32-bits ARM Cortex-M3)" 설치 후 보드로 "Arduino Due" 선택

  안전 경고 (반드시 확인)
  -------------
  - 이 컨트롤러는 정격 400A / 최대 1000A입니다. 최초 테스트는 반드시 바퀴가 지면에서
    뜬 상태(스탠드에 거치)에서, 아래 MAX_TARGET_CURRENT_A 를 낮은 값(예: 5~10A)으로
    설정하고 진행하세요.
  - ENABLE_PIN 은 최소한의 안전 인터록 예시입니다. 실제 차량에는 별도의 비상정지(E-STOP),
    브레이크 연동, 웨이크업 시퀀스 등을 반드시 추가해야 합니다.
  - EZkontrol 앱(Bluetooth)에서 Setting > Communication > "CAN communication protocol"
    값이 2(250Kbps, 본 코드 기본값)로 설정되어 있는지 반드시 확인하세요. 컨트롤러가
    여러 대라면 Part number(주소, 기본 239=0xEF)도 확인하세요.

  통신 개요 (문서 4장 Handshake Protocol)
  -------------
  1) 컨트롤러(MCU) 전원 인가 후 0x1801D0EF 로 8바이트 전부 0x55 를 50ms 주기로 반복 전송.
  2) VCU(본 아두이노)가 이를 감지하면 0x0C01EFD0 로 8바이트 전부 0xAA 를 응답.
  3) 핸드셰이크가 성립하면 MCU는 정상 텔레메트리(전압/전류/속도, 온도/상태/에러)를 보내기
     시작하고, VCU는 이제부터 0xAA 대신 실제 제어 명령(목표 전류/속도)을 50ms 주기로 전송.
  4) MCU가 VCU 명령을 10회 연속 못 받거나 Life signal이 5회 연속 틀리면 통신 실패로 보고
     핸드셰이크부터 재시작 -> 이 스케치도 동일하게 텔레메트리 타임아웃 시 재핸드셰이크 상태로 복귀.
*/

#include <due_can.h>

// ================= 사용자 설정값 =================
#define CAN_PORT              Can0            // 배선한 CAN 컨트롤러: Can0 또는 Can1
#define CAN_BAUDRATE          CAN_BPS_250K    // EZkontrol 앱의 CAN communication protocol=2(250K) 와 반드시 일치시킬 것

// 문서 3장 Messages 에 정의된 확장(29비트) CAN ID (VCU=0xD0, MCU1=0xEF 기준, 그대로 사용)
const uint32_t ID_CMD  = 0x0C01EFD0UL;  // VCU -> MCU : 제어 명령 (Message I)
const uint32_t ID_TLM1 = 0x1801D0EFUL;  // MCU -> VCU : BUS전압/BUS전류/상전류/속도 (Message II Part I)
const uint32_t ID_TLM2 = 0x1802D0EFUL;  // MCU -> VCU : 온도/상태/에러 (Message II Part II)

const uint32_t TX_PERIOD_MS  = 50;   // 문서 규정 폴링 주기 (50ms)
const uint32_t RX_TIMEOUT_MS = 500;  // 이 시간 이상 MCU 텔레메트리가 없으면 통신 두절로 판단

const float MAX_TARGET_CURRENT_A = 10.0;    // 안전상 초기 상한(정격 400A/최대 1000A) - 벤치 테스트 후 단계적으로만 상향
const float MAX_TARGET_SPEED_RPM = 3000.0;  // 토크제어 모드에서는 이 값이 "속도 상한"으로 사용됨(문서 5-(3))

const int THROTTLE_PIN = A0;  // 스로틀 아날로그 입력(0~5V, 0~1023) - ROS2 시리얼 명령이 없을 때만 사용(단독 벤치 테스트용)
const int ENABLE_PIN   = 22;  // 주행 허용 스위치. INPUT_PULLUP 이므로 미연결/OFF 시 기본적으로 "정지"(안전측 기본값)

// ROS2(ev_arduino_bridge 노드)와의 시리얼 통신 - USB(Serial)를 디버그 출력과 공유해서 사용.
// 프로토콜: "C,<enable:0|1>,<throttle_percent:float>,<steer_deg:float>,<brake_percent:float>\n"
// steer_deg/brake_percent는 이 스케치(후륜구동계 전용)에서는 아직 사용하지 않음 - 조향/브레이크는
// L7SA 서보드라이브(Modbus RTU) 대상이라 별도 인터페이스가 필요함 (TODO, firmware/ 참고).
const uint32_t SERIAL_CMD_TIMEOUT_MS = 300;  // 이 시간 동안 새 시리얼 명령이 없으면 시리얼 제어 비활성 -> 정지 또는 아날로그 폴백

// ================= 내부 상태 =================
enum LinkState { WAIT_HANDSHAKE, RUNNING };
LinkState linkState = WAIT_HANDSHAKE;

uint8_t lifeSignalTx = 0;
uint32_t lastTxMs = 0;
uint32_t lastRxMs = 0;

bool serialEnable = false;
float serialThrottlePercent = 0.0;
float serialSteerDeg = 0.0;
float serialBrakePercent = 0.0;
uint32_t lastSerialRxMs = 0;
String serialLineBuf = "";

struct Telemetry {
  float busVoltage_V = 0;
  float busCurrent_A = 0;
  float phaseCurrent_A = 0;
  float speed_RPM = 0;
  int controllerTemp_C = 0;
  int motorTemp_C = 0;
  bool running = false;
  bool speedMode = false;
  uint8_t errByte3 = 0, errByte4 = 0, errByte5 = 0;
} tlm;

// ================= 물리값 <-> 전송값 변환 (문서 5-(1) 공식) =================
uint16_t encodeCurrent(float amps) {
  amps = constrain(amps, -3200.0, 3200.0);
  return (uint16_t)((amps - (-3200.0)) / 0.1);
}
uint16_t encodeSpeed(float rpm) {
  rpm = constrain(rpm, -32000.0, 32000.0);
  return (uint16_t)((rpm - (-32000.0)) / 1.0);
}
float decodeCurrent01A(uint16_t raw) { return raw * 0.1 - 3200.0; }
float decodeSpeedRpm(uint16_t raw)   { return (float)raw - 32000.0; }
float decodeVoltage01V(uint16_t raw) { return raw * 0.1; }
int   decodeTempC(uint8_t raw)       { return (int)raw - 40; }

// ================= CAN 프레임 송신 =================
void sendFrame(uint32_t id, const uint8_t *data) {
  CAN_FRAME f;
  f.id = id;
  f.extended = true;
  f.rtr = 0;
  f.length = 8;
  for (uint8_t i = 0; i < 8; i++) f.data.byte[i] = data[i];
  CAN_PORT.sendFrame(f);
}

void sendHandshakeAck() {
  uint8_t d[8];
  for (uint8_t i = 0; i < 8; i++) d[i] = 0xAA;
  sendFrame(ID_CMD, d);
}

// run=false 또는 targetCurrentA<=0 이면 정지(HALTED) 명령이 전송됨
void sendCommand(float targetCurrentA, float targetSpeedRPM, bool run, bool speedMode) {
  uint16_t curRaw = encodeCurrent(targetCurrentA);
  uint16_t spdRaw = encodeSpeed(targetSpeedRPM);

  uint8_t d[8];
  d[0] = (uint8_t)(curRaw & 0xFF);
  d[1] = (uint8_t)(curRaw >> 8);
  d[2] = (uint8_t)(spdRaw & 0xFF);
  d[3] = (uint8_t)(spdRaw >> 8);
  d[4] = (run ? 0x01 : 0x00) | (speedMode ? 0x02 : 0x00);  // bit0: RUN, bit1: 0=토크제어 1=속도제어
  d[5] = 0x00;  // reserved
  d[6] = 0x00;  // reserved
  d[7] = lifeSignalTx++;  // Life signal, 0~0xFF 순환 카운터

  sendFrame(ID_CMD, d);
}

// ================= 스로틀 입력 (데모용, 실차량은 안전 인증된 센서/로직으로 교체) =================
float readThrottleCurrentCommand() {
  int raw = analogRead(THROTTLE_PIN);              // 0~1023
  if (raw < 20) return 0.0;                          // 데드밴드
  float pct = (raw - 20) / (float)(1023 - 20);
  pct = constrain(pct, 0.0, 1.0);
  return pct * MAX_TARGET_CURRENT_A;
}

// ================= ROS2 시리얼 명령 파싱 =================
void parseSerialLine(const String &line) {
  if (!line.startsWith("C,")) return;  // 디버그 출력 등 다른 줄은 무시

  int idx1 = line.indexOf(',', 2);
  int idx2 = line.indexOf(',', idx1 + 1);
  int idx3 = line.indexOf(',', idx2 + 1);
  if (idx1 < 0 || idx2 < 0 || idx3 < 0) return;  // 형식이 안 맞으면 버림

  serialEnable = line.substring(2, idx1).toInt() != 0;
  serialThrottlePercent = line.substring(idx1 + 1, idx2).toFloat();
  serialSteerDeg = line.substring(idx2 + 1, idx3).toFloat();
  serialBrakePercent = line.substring(idx3 + 1).toFloat();
  lastSerialRxMs = millis();
}

void pollSerialCommand() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseSerialLine(serialLineBuf);
      serialLineBuf = "";
    } else if (c != '\r') {
      serialLineBuf += c;
      if (serialLineBuf.length() > 64) serialLineBuf = "";  // 비정상적으로 긴 줄 방지
    }
  }
}

// ================= 에러 비트 이름표 (문서 3-III ERROR 필드) =================
const char* ERR3_NAMES[8] = { "Overcurrent", "Overload", "Overvoltage", "Undervoltage",
                               "ControllerOverheat", "MotorOverheat", "MotorStalled", "MotorOutOfPhase" };
const char* ERR4_NAMES[8] = { "MotorSensor", "MotorAuxSensor", "EncoderMisaligned", "AntiRunawayEngaged",
                               "MainAccelerator", "AuxAccelerator", "Precharge", "DCContactor" };
const char* ERR5_NAMES[8] = { "PowerValve", "CurrentSensor", "AutoTune", "RS485", "CAN", "Software", "-", "-" };

void printErrors() {
  for (uint8_t i = 0; i < 8; i++) {
    if (tlm.errByte3 & (1 << i)) { Serial.print("[FAULT] "); Serial.println(ERR3_NAMES[i]); }
    if (tlm.errByte4 & (1 << i)) { Serial.print("[FAULT] "); Serial.println(ERR4_NAMES[i]); }
    if (tlm.errByte5 & (1 << i)) { Serial.print("[FAULT] "); Serial.println(ERR5_NAMES[i]); }
  }
}

// ================= CAN 수신 처리 =================
void handleRxFrame(CAN_FRAME &f) {
  lastRxMs = millis();

  if (f.id == ID_TLM1) {
    bool isHandshakeByte = true;
    for (uint8_t i = 0; i < 8; i++) {
      if (f.data.byte[i] != 0x55) { isHandshakeByte = false; break; }
    }
    if (isHandshakeByte) {
      linkState = WAIT_HANDSHAKE;  // 컨트롤러가 아직 핸드셰이크 대기 중 -> 다음 TX 주기에 0xAA 응답
      return;
    }

    linkState = RUNNING;
    uint16_t vRaw      = f.data.byte[0] | (f.data.byte[1] << 8);
    uint16_t iBusRaw   = f.data.byte[2] | (f.data.byte[3] << 8);
    uint16_t iPhaseRaw = f.data.byte[4] | (f.data.byte[5] << 8);
    uint16_t spdRaw    = f.data.byte[6] | (f.data.byte[7] << 8);

    tlm.busVoltage_V   = decodeVoltage01V(vRaw);
    tlm.busCurrent_A   = decodeCurrent01A(iBusRaw);
    tlm.phaseCurrent_A = decodeCurrent01A(iPhaseRaw);
    tlm.speed_RPM      = decodeSpeedRpm(spdRaw);
  }
  else if (f.id == ID_TLM2) {
    tlm.controllerTemp_C = decodeTempC(f.data.byte[0]);
    tlm.motorTemp_C       = decodeTempC(f.data.byte[1]);
    tlm.running           = f.data.byte[2] & 0x01;
    tlm.speedMode          = f.data.byte[2] & 0x02;
    tlm.errByte3 = f.data.byte[3];
    tlm.errByte4 = f.data.byte[4];
    tlm.errByte5 = f.data.byte[5];
  }
}

// ================= setup / loop =================
void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, INPUT_PULLUP);  // 안전측 기본값: 스위치 미연결이면 주행 불가 상태로 유지됨

  CAN_PORT.begin(CAN_BAUDRATE);
  CAN_PORT.watchFor(ID_TLM1);
  CAN_PORT.watchFor(ID_TLM2);

  lastRxMs = millis();
  Serial.println("EZkontrol B481000 CAN control start");
}

void loop() {
  pollSerialCommand();

  CAN_FRAME f;
  while (CAN_PORT.rx_avail()) {
    CAN_PORT.read(f);
    handleRxFrame(f);
  }

  // 텔레메트리 타임아웃 -> 재핸드셰이크 상태로 복귀 (문서 4장 "10회 연속 실패 시 재시작"에 대응하는 VCU측 안전 처리)
  if (linkState == RUNNING && (millis() - lastRxMs > RX_TIMEOUT_MS)) {
    linkState = WAIT_HANDSHAKE;
    Serial.println("[WARN] MCU telemetry timeout -> re-handshake");
  }

  uint32_t now = millis();
  if (now - lastTxMs >= TX_PERIOD_MS) {
    lastTxMs = now;

    if (linkState == WAIT_HANDSHAKE) {
      sendHandshakeAck();
    } else {
      bool hwEnable = (digitalRead(ENABLE_PIN) == LOW);  // 배선 예: 스위치 ON 시 GND로 당겨짐 = LOW = 주행 허용
      bool serialActive = (now - lastSerialRxMs < SERIAL_CMD_TIMEOUT_MS);

      float targetCurrent;
      bool run;
      if (serialActive) {
        // ROS2(ev_arduino_bridge)가 보낸 명령 사용. 하드웨어 스위치와 시리얼 enable 둘 다 필요.
        bool enable = hwEnable && serialEnable;
        targetCurrent = enable ? (serialThrottlePercent / 100.0) * MAX_TARGET_CURRENT_A : 0.0;
        run = enable && (targetCurrent > 0.1);
      } else {
        // ROS2 연결이 없으면 아날로그 스로틀로 단독 벤치 테스트 가능
        targetCurrent = hwEnable ? readThrottleCurrentCommand() : 0.0;
        run = hwEnable && (targetCurrent > 0.1);
      }

      sendCommand(targetCurrent, MAX_TARGET_SPEED_RPM, run, /*speedMode=*/false);
    }
  }

  static uint32_t lastPrintMs = 0;
  if (now - lastPrintMs >= 500) {
    lastPrintMs = now;
    Serial.print("Link="); Serial.print(linkState == RUNNING ? "RUNNING" : "HANDSHAKE");
    Serial.print(" V=");     Serial.print(tlm.busVoltage_V);
    Serial.print(" IBus=");  Serial.print(tlm.busCurrent_A);
    Serial.print(" IPhase="); Serial.print(tlm.phaseCurrent_A);
    Serial.print(" RPM=");   Serial.print(tlm.speed_RPM);
    Serial.print(" Tctrl="); Serial.print(tlm.controllerTemp_C);
    Serial.print(" Tmot=");  Serial.print(tlm.motorTemp_C);
    Serial.print(" Run=");   Serial.println(tlm.running);
    printErrors();
  }
}
