// dxl_setup.ino — 팔 Dynamixel 사전 설정 도구 (Firmware 팀). OpenCR 에 올리고 USB 시리얼로 한 줄씩 명령한다.
//   목적: 공장 출하 모터 (보통 ID 1 · 57600 bps) 를 펌웨어 설정 (config.h: ID 11~16 · 1 Mbps · Protocol 2.0) 으로 맞춘다.
//   토크를 켜는 명령은 없다. EEPROM 쓰기 (ID · 통신 속도) 는 토크가 꺼진 모터에만 한다.
//   같은 ID 모터가 버스에 여러 대 있으면 응답이 겹치고, 그 ID 로 보낸 쓰기는 모두에게 적용된다 → 모터를 한 대씩 연결해 설정한다.
//
// 명령 (한 줄 + 엔터):
//   s              모든 통신 속도 × ID 0~252 탐색 (Protocol 2.0, 약 20 s)
//   p <baud>       이후 명령에 쓸 포트 통신 속도 (시작 1000000)
//   r <id>         모델 · 펌웨어 · 토크 · 모드 · Homing Offset · Return Delay · HW 오류 · 전압 · 현재 위치
//   x <id>         진단용 Control Table 덤프 (읽기만): Drive Mode · 한계값 (PWM · 전류 · 속도 · 위치) · 게인 · PWM · 전류 · 온도
//   l <id> <0|1>   LED 끄기 / 켜기 (어느 모터인지 찾기)
//   i <id> <new>   ID 변경 (EEPROM)
//   b <id> <baud>  통신 속도 변경 (EEPROM). 바꾼 뒤 포트도 그 속도로 바꿔 다시 확인한다
//   m <id> <min> <max>  위치 한계 변경 (EEPROM Min/Max Position Limit, tick 0~4095). 쓴 뒤 다시 읽어 확인
//   h              도움말
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL  Serial3   // opencr_firmware/config.h 와 같게
#define DXL_DIR_PIN 84
static const float PROTOCOL = 2.0f;
static const uint32_t BAUDS[] = {57600, 1000000, 115200, 2000000, 3000000, 4000000, 9600};
static const int N_BAUDS = sizeof(BAUDS) / sizeof(BAUDS[0]);

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
static uint32_t g_baud = 1000000;

static void usePort(uint32_t baud)
{
  dxl.begin(baud);
  dxl.setPortProtocolVersion(PROTOCOL);
  g_baud = baud;
}

static bool knownBaud(uint32_t baud)
{
  for (int i = 0; i < N_BAUDS; ++i) {
    if (BAUDS[i] == baud) {return true;}
  }
  return false;
}

// 읽기에 성공했고 값이 0 일 때만 "토크 꺼짐" (읽기 실패도 0 을 돌려주므로 오류 코드로 구분)
static bool torqueIsOff(uint8_t id)
{
  const int32_t t = dxl.readControlTableItem(TORQUE_ENABLE, id);
  return dxl.getLastLibErrCode() == DXL_LIB_OK && t == 0;
}

static void printItem(const char * name, uint8_t item, uint8_t id)
{
  const int32_t v = dxl.readControlTableItem(item, id);
  Serial.print("  ");
  Serial.print(name);
  Serial.print('=');
  if (dxl.getLastLibErrCode() == DXL_LIB_OK) {Serial.println(v);} else {Serial.println("읽기 실패");}
}

static void scan()
{
  const uint32_t prev = g_baud;
  int found = 0;
  for (int b = 0; b < N_BAUDS; ++b) {
    usePort(BAUDS[b]);
    Serial.print("SCAN baud=");
    Serial.println(BAUDS[b]);
    for (int id = 0; id < DXL_BROADCAST_ID; ++id) {
      if (!dxl.ping(id)) {continue;}
      Serial.print("  FOUND id=");
      Serial.print(id);
      Serial.print(" model=");
      Serial.print(dxl.getModelNumber(id));
      Serial.print(" baud=");
      Serial.println(BAUDS[b]);
      ++found;
    }
  }
  usePort(prev);
  Serial.print("SCAN DONE found=");
  Serial.print(found);
  Serial.print(" (포트는 다시 ");
  Serial.print(prev);
  Serial.println(")");
}

static void info(uint8_t id)
{
  if (!dxl.ping(id)) {Serial.println("ERR 응답 없음 (포트 통신 속도 확인: p <baud>)"); return;}
  Serial.print("INFO id=");
  Serial.print(id);
  Serial.print(" baud=");
  Serial.print(g_baud);
  Serial.print(" model=");
  Serial.println(dxl.getModelNumber(id));
  printItem("firmware", FIRMWARE_VERSION, id);
  printItem("torque_enable", TORQUE_ENABLE, id);
  printItem("operating_mode", OPERATING_MODE, id);
  printItem("homing_offset", HOMING_OFFSET, id);
  printItem("return_delay", RETURN_DELAY_TIME, id);
  printItem("hw_error", HARDWARE_ERROR_STATUS, id);
  printItem("bus_watchdog", BUS_WATCHDOG, id);
  printItem("input_voltage_x0.1V", PRESENT_INPUT_VOLTAGE, id);
  printItem("present_position_tick", PRESENT_POSITION, id);
}

// XM430 Control Table (e-Manual) 의 진단 항목 — 주소 · 길이 · 부호
struct RawItem {const char * name; uint16_t addr; uint8_t len; bool is_signed;};
static const RawItem RAW_ITEMS[] = {
  {"drive_mode", 10, 1, false},          {"operating_mode", 11, 1, false},
  {"temperature_limit", 31, 1, false},   {"max_voltage_x0.1V", 32, 2, false},  {"min_voltage_x0.1V", 34, 2, false},
  {"pwm_limit", 36, 2, false},           {"current_limit", 38, 2, false},      {"velocity_limit", 44, 4, false},
  {"max_position_limit", 48, 4, false},  {"min_position_limit", 52, 4, false}, {"shutdown", 63, 1, false},
  {"torque_enable", 64, 1, false},       {"hw_error", 70, 1, false},
  {"pos_d_gain", 80, 2, false},          {"pos_i_gain", 82, 2, false},         {"pos_p_gain", 84, 2, false},
  {"goal_pwm", 100, 2, true},            {"goal_position", 116, 4, true},
  {"present_pwm", 124, 2, true},         {"present_current", 126, 2, true},    {"present_position", 132, 4, true},
  {"present_temperature", 146, 1, false},
};

// 읽기 전용: 원래 바이트를 길이 · 부호대로 해석 (readControlTableItem 의 1 바이트 부호 확장 · 길이 차이를 피함)
static void dumpRaw(uint8_t id)
{
  if (!dxl.ping(id)) {Serial.println("ERR 응답 없음 (포트 통신 속도 확인: p <baud>)"); return;}
  Serial.print("DUMP id=");
  Serial.println(id);
  for (const RawItem & it : RAW_ITEMS) {
    uint8_t buf[4] = {0, 0, 0, 0};
    Serial.print("  ");
    Serial.print(it.name);
    Serial.print('=');
    if (dxl.read(id, it.addr, it.len, buf, sizeof(buf)) != it.len) {Serial.println("읽기 실패"); continue;}
    const uint32_t u = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    if (!it.is_signed) {Serial.println(u);}
    else if (it.len == 2) {Serial.println((int16_t)u);}
    else {Serial.println((int32_t)u);}
  }
}

static void changeLimits(uint8_t id, uint32_t lo, uint32_t hi)
{
  if (lo >= hi || hi > 4095) {Serial.println("ERR 0 <= min < max <= 4095"); return;}
  if (!dxl.ping(id)) {Serial.println("ERR 응답 없음"); return;}
  if (!torqueIsOff(id)) {Serial.println("ERR 토크가 켜져 있거나 읽기 실패 — EEPROM 을 바꾸지 않음"); return;}
  if (!dxl.writeControlTableItem(MIN_POSITION_LIMIT, id, (int32_t)lo) || !dxl.writeControlTableItem(MAX_POSITION_LIMIT, id, (int32_t)hi)) {
    Serial.println("ERR 쓰기 실패 — x 로 다시 확인");
    return;
  }
  const int32_t rlo = dxl.readControlTableItem(MIN_POSITION_LIMIT, id);
  const int32_t rhi = dxl.readControlTableItem(MAX_POSITION_LIMIT, id);
  if (rlo == (int32_t)lo && rhi == (int32_t)hi) {
    Serial.print("OK id ");
    Serial.print(id);
    Serial.print(" limit ");
    Serial.print(rlo);
    Serial.print(" ~ ");
    Serial.println(rhi);
  } else {
    Serial.println("ERR 확인 실패 — x 로 다시 확인");
  }
}

static void changeId(uint8_t id, uint32_t nid)
{
  if (nid >= DXL_BROADCAST_ID) {Serial.println("ERR 새 ID 는 0~252"); return;}
  if (!dxl.ping(id)) {Serial.println("ERR 응답 없음"); return;}
  if (dxl.ping((uint8_t)nid)) {Serial.println("ERR 새 ID 를 이미 쓰는 모터가 있음"); return;}
  if (!torqueIsOff(id)) {Serial.println("ERR 토크가 켜져 있거나 읽기 실패 — EEPROM 을 바꾸지 않음"); return;}
  if (!dxl.setID(id, (uint8_t)nid)) {Serial.println("ERR ID 쓰기 실패"); return;}
  delay(50);
  if (dxl.ping((uint8_t)nid) && !dxl.ping(id)) {
    Serial.print("OK id ");
    Serial.print(id);
    Serial.print(" -> ");
    Serial.println(nid);
  } else {
    Serial.println("ERR 확인 실패 — s 로 다시 탐색");
  }
}

static void changeBaud(uint8_t id, uint32_t nb)
{
  if (!knownBaud(nb)) {Serial.println("ERR 지원 통신 속도: 9600 57600 115200 1000000 2000000 3000000 4000000"); return;}
  if (!dxl.ping(id)) {Serial.println("ERR 응답 없음"); return;}
  if (!torqueIsOff(id)) {Serial.println("ERR 토크가 켜져 있거나 읽기 실패 — EEPROM 을 바꾸지 않음"); return;}
  if (!dxl.setBaudrate(id, nb)) {Serial.println("ERR 통신 속도 쓰기 실패"); return;}
  delay(50);
  usePort(nb);
  if (dxl.ping(id)) {
    Serial.print("OK id ");
    Serial.print(id);
    Serial.print(" baud -> ");
    Serial.print(nb);
    Serial.println(" (포트도 이 속도로 바뀜)");
  } else {
    Serial.println("ERR 새 통신 속도에서 응답 없음 — s 로 다시 탐색");
  }
}

static void help()
{
  Serial.println("dxl_setup — s 탐색 | p <baud> | r <id> | x <id> | l <id> <0|1> | i <id> <new> | b <id> <baud> | m <id> <min> <max> | h");
  Serial.println("  토크 ON 명령 없음. EEPROM 쓰기 (i, b) 는 모터를 한 대씩 연결한 상태에서만. m 은 ID 가 겹치지 않으면 체인 그대로.");
  Serial.print("  현재 포트 baud=");
  Serial.println(g_baud);
}

static void handle(char * line)
{
  char cmd = 0;
  unsigned long a = 0, b = 0, c = 0;
  const int n = sscanf(line, " %c %lu %lu %lu", &cmd, &a, &b, &c);
  if (n < 1) {return;}
  switch (cmd) {
    case 's': scan(); break;
    case 'p':
      if (n == 2 && knownBaud(a)) {usePort(a); Serial.print("OK port baud="); Serial.println(a);} else {Serial.println("ERR p <baud>");}
      break;
    case 'r': if (n == 2) {info((uint8_t)a);} else {Serial.println("ERR r <id>");} break;
    case 'x': if (n == 2) {dumpRaw((uint8_t)a);} else {Serial.println("ERR x <id>");} break;
    case 'l':
      if (n == 3) {
        const bool ok = b ? dxl.ledOn((uint8_t)a) : dxl.ledOff((uint8_t)a);
        Serial.println(ok ? "OK led" : "ERR led");
      } else {
        Serial.println("ERR l <id> <0|1>");
      }
      break;
    case 'i': if (n == 3) {changeId((uint8_t)a, b);} else {Serial.println("ERR i <id> <new>");} break;
    case 'b': if (n == 3) {changeBaud((uint8_t)a, b);} else {Serial.println("ERR b <id> <baud>");} break;
    case 'm': if (n == 4) {changeLimits((uint8_t)a, b, c);} else {Serial.println("ERR m <id> <min> <max>");} break;
    default: help(); break;
  }
}

void setup()
{
  Serial.begin(115200);   // USB CDC — 보드레이트 무관
  usePort(g_baud);
  help();
}

void loop()
{
  static char line[48];
  static int len = 0;
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        line[len] = 0;
        handle(line);
        len = 0;
      }
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}
