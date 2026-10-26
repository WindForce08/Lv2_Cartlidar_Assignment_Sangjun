// Arduino 코어 대역 (테스트 더블) — opencr_firmware.ino 가 쓰는 것만. sil_firmware.cpp 가 역할별 namespace 안에서 include.
//   g_board (SilBoard) 가 먼저 선언되어 있어야 한다.
static const int LOW = 0;
static const int HIGH = 1;
static const int INPUT_PULLUP = 2;

inline uint32_t micros() {return static_cast<uint32_t>(g_board.now_us);}
inline uint32_t millis() {return static_cast<uint32_t>(g_board.now_us / 1000);}
inline void pinMode(int, int) {}
inline int digitalRead(int) {return g_board.estop_pin_low ? LOW : HIGH;}
inline float getPowerInVoltage() {return g_board.vin;}

// USB CDC (HOST_SERIAL = Serial)
class SilHostSerial
{
public:
  void begin(unsigned long) {}
  int available() {return static_cast<int>(g_board.host_rx.size());}
  int read()
  {
    if (g_board.host_rx.empty()) {return -1;}
    const uint8_t b = g_board.host_rx.front();
    g_board.host_rx.pop_front();
    return b;
  }
  size_t write(const uint8_t * p, size_t n)
  {
    g_board.host_tx.insert(g_board.host_tx.end(), p, p + n);
    return n;
  }
};

// Dynamixel 포트 (DXL_SERIAL = Serial3) — 버스 동작은 Dynamixel2Arduino 대역이 직접 흉내
class SilDxlSerial {};

SilHostSerial Serial;
SilDxlSerial Serial3;
