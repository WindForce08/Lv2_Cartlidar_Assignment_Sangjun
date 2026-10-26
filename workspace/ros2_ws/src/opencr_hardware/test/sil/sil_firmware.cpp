// 실제 opencr_firmware.ino 를 역할별 namespace 안에 그대로 넣어 컴파일한다 (SIL).
//   빌드 정의: SIL_NS (namespace 이름) · SIL_ACCESSOR (보드 함수 이름) · OPENCR_BOARD_ROLE · FIRMWARE_INO (ino 절대 경로)
//   include 경로: test/sil/stubs (Dynamixel2Arduino.h · IMU.h 대역) · opencr_firmware (config.h 등)
// 표준 헤더는 namespace 밖에서 먼저 열어 두어 ino · 프로토콜 헤더 안의 include 가 다시 열리지 않게 한다.
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "sil_board.hpp"

namespace SIL_NS
{
SilBoard g_board;
#include "sil_env.hpp"
#include "Dynamixel2Arduino.h"
#include "IMU.h"
#include FIRMWARE_INO
}  // namespace SIL_NS

SilBoard & SIL_ACCESSOR()
{
  SIL_NS::g_board.setup = &SIL_NS::setup;
  SIL_NS::g_board.loop = &SIL_NS::loop;
  return SIL_NS::g_board;
}
