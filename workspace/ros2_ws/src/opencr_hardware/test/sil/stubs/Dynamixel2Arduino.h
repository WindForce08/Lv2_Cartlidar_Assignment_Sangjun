// Dynamixel2Arduino 0.8.2 대역 (테스트 더블) — opencr_firmware.ino 가 쓰는 API 만, 같은 이름 · 같은 구조체 필드로.
//   동작 모델 (라이브러리 소스를 읽고 흉내 냄, 실제와 같다고 보장하지 않음):
//   - syncRead: ID 순서대로 받다가 처음 응답 없는 모터에서 멈춘다. 멈춘 칸 버퍼에는 쓰레기 값이 들어가고,
//     timeout_ms(기본 10 ms) 만큼 시계가 흐른다 (master.cpp Master::syncRead · rxStatusPacket).
//   - readControlTableItem: 응답 없으면 0 을 돌려주고 timeout(기본 100 ms) 만큼 시계가 흐른다. getLastLibErrCode 로 구분.
//   - writeControlTableItem: 브로드캐스트(0xFE)는 응답을 기다리지 않고 모든 모터에 적용.
//   - syncWrite: 토크 ON 모터는 목표에 즉시 도달 (motors_follow_goals) — 시험이 max_step (속도) · stop_min/max (막힘) 을 주면 그만큼만.
//     Bus Watchdog 만료 동작은 흉내 내지 않음.
//     Min/Max Position Limit 밖 Goal Position 은 모터가 버린다 (e-Manual Data Limit — SyncWrite 라 응답 없음).
#ifndef SIL_DYNAMIXEL2ARDUINO_H_
#define SIL_DYNAMIXEL2ARDUINO_H_

const int DXL_BROADCAST_ID = 0xFE;
enum OperatingMode { OP_CURRENT = 0, OP_VELOCITY = 1, OP_POSITION = 3, OP_EXTENDED_POSITION = 4 };
enum DXLLibErrorCode { DXL_LIB_OK = 0, DXL_LIB_ERROR_TIMEOUT = 3 };   // 실제는 dxl_c/protocol.h (TIMEOUT 값은 대역용)

namespace ControlTableItem
{
enum ControlTableItemIndex { HOMING_OFFSET, RETURN_DELAY_TIME, BUS_WATCHDOG, PROFILE_VELOCITY, PROFILE_ACCELERATION,
  HARDWARE_ERROR_STATUS, TORQUE_ENABLE, GOAL_POSITION, GOAL_VELOCITY, OPERATING_MODE, MIN_POSITION_LIMIT, MAX_POSITION_LIMIT,
  POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN, PWM_LIMIT };
}

namespace DYNAMIXEL
{
struct InfoSyncBulkBuffer_t { uint8_t * p_buf; uint16_t buf_capacity; uint16_t gen_length; bool is_completed; } __attribute__((packed));
struct XELInfoSyncRead_t { uint8_t * p_recv_buf; uint8_t id; uint8_t error; } __attribute__((packed));
struct InfoSyncReadInst_t
{
  uint16_t addr;
  uint16_t addr_length;
  XELInfoSyncRead_t * p_xels;
  uint8_t xel_count;
  bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
} __attribute__((packed));
struct XELInfoSyncWrite_t { uint8_t * p_data; uint8_t id; } __attribute__((packed));
struct InfoSyncWriteInst_t
{
  uint16_t addr;
  uint16_t addr_length;
  XELInfoSyncWrite_t * p_xels;
  uint8_t xel_count;
  bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
} __attribute__((packed));
}  // namespace DYNAMIXEL

class Dynamixel2Arduino
{
public:
  template<class Port>
  Dynamixel2Arduino(Port &, int) {}

  void begin(unsigned long) {}
  bool setPortProtocolVersion(float) {return true;}
  bool ping(uint8_t id) {return find(id) != nullptr;}
  uint16_t getModelNumber(uint8_t id)
  {
    SilMotor * m = find(id);
    return m ? m->model : 0;
  }
  bool torqueOff(uint8_t id) {return writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, 0);}
  bool setOperatingMode(uint8_t id, uint8_t mode) {return writeControlTableItem(ControlTableItem::OPERATING_MODE, id, mode);}

  int32_t getLastLibErrCode() const {return last_err_;}

  int32_t readControlTableItem(uint8_t item, uint8_t id, uint32_t timeout = 100)
  {
    SilMotor * m = find(id);
    if (!m) {
      g_board.now_us += timeout * 1000ull;
      ++g_board.failed_reads;
      last_err_ = DXL_LIB_ERROR_TIMEOUT;
      return 0;
    }
    last_err_ = DXL_LIB_OK;
    switch (item) {
      case ControlTableItem::HOMING_OFFSET: return m->homing_offset;
      case ControlTableItem::HARDWARE_ERROR_STATUS: return m->hw_error;
      case ControlTableItem::TORQUE_ENABLE: return m->torque ? 1 : 0;
      case ControlTableItem::BUS_WATCHDOG: return m->watchdog;
      case ControlTableItem::OPERATING_MODE: return m->mode;
      case ControlTableItem::MIN_POSITION_LIMIT: return m->min_position_limit;
      case ControlTableItem::MAX_POSITION_LIMIT: return m->max_position_limit;
      case ControlTableItem::POSITION_P_GAIN: return m->pos_p_gain;
      case ControlTableItem::POSITION_I_GAIN: return m->pos_i_gain;
      case ControlTableItem::POSITION_D_GAIN: return m->pos_d_gain;
      case ControlTableItem::PWM_LIMIT: return m->pwm_limit;
      default: return 0;
    }
  }

  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t timeout = 100)
  {
    g_board.writes.push_back(SilWrite{item, id, value});
    if (id == DXL_BROADCAST_ID) {
      for (auto & kv : g_board.motors) {
        if (kv.second.present) {apply(kv.second, item, value);}
      }
      return true;
    }
    SilMotor * m = find(id);
    if (!m) {
      g_board.now_us += timeout * 1000ull;
      return false;
    }
    apply(*m, item, value);
    return true;
  }

  uint8_t syncRead(DYNAMIXEL::InfoSyncReadInst_t * p, uint32_t timeout_ms = 10)
  {
    uint8_t recv = 0;
    for (uint8_t i = 0; i < p->xel_count; ++i) {
      DYNAMIXEL::XELInfoSyncRead_t & x = p->p_xels[i];
      SilMotor * m = find(x.id);
      if (!m) {
        memset(x.p_recv_buf, 0x5A, p->addr_length);   // 끊긴 칸: 다른 패킷 · 일부만 받은 바이트
        g_board.now_us += timeout_ms * 1000ull;
        break;
      }
      // addr 126 부터: Present Current(2) · Velocity(4) · Position(4)
      memcpy(x.p_recv_buf + 0, &m->current, 2);
      memcpy(x.p_recv_buf + 2, &m->velocity, 4);
      memcpy(x.p_recv_buf + 6, &m->position, 4);
      x.error = 0;
      ++recv;
    }
    return recv;
  }

  bool syncWrite(DYNAMIXEL::InfoSyncWriteInst_t * p)
  {
    ++g_board.sync_writes;
    for (uint8_t i = 0; i < p->xel_count; ++i) {
      DYNAMIXEL::XELInfoSyncWrite_t & x = p->p_xels[i];
      SilMotor * m = find(x.id);
      if (!m) {continue;}
      int32_t v;
      memcpy(&v, x.p_data, 4);
      if (p->addr == 116) {   // Goal Position
        if (v < m->min_position_limit || v > m->max_position_limit) {continue;}
        m->goal_position = v;
        if (m->torque && g_board.motors_follow_goals) {
          int32_t p = v;
          if (m->max_step > 0) {
            if (p > m->position + m->max_step) {p = m->position + m->max_step;}
            if (p < m->position - m->max_step) {p = m->position - m->max_step;}
          }
          if (p > m->stop_max) {p = m->stop_max;}
          if (p < m->stop_min) {p = m->stop_min;}
          m->position = p;
        }
      } else if (p->addr == 104) {   // Goal Velocity
        m->goal_velocity = v;
        if (m->torque && g_board.motors_follow_goals) {m->velocity = v;}
      }
    }
    return true;
  }

private:
  int32_t last_err_ = DXL_LIB_OK;

  static SilMotor * find(uint8_t id)
  {
    auto it = g_board.motors.find(id);
    return (it == g_board.motors.end() || !it->second.present) ? nullptr : &it->second;
  }
  static void apply(SilMotor & m, uint8_t item, int32_t v)
  {
    switch (item) {
      case ControlTableItem::TORQUE_ENABLE: m.torque = v != 0; break;
      case ControlTableItem::BUS_WATCHDOG: m.watchdog = v; break;
      case ControlTableItem::OPERATING_MODE: m.mode = static_cast<uint8_t>(v); break;
      case ControlTableItem::HOMING_OFFSET: m.homing_offset = v; break;
      case ControlTableItem::POSITION_P_GAIN: if (!m.ignore_gain_writes) {m.pos_p_gain = v;} break;
      case ControlTableItem::POSITION_I_GAIN: if (!m.ignore_gain_writes) {m.pos_i_gain = v;} break;
      case ControlTableItem::POSITION_D_GAIN: if (!m.ignore_gain_writes) {m.pos_d_gain = v;} break;
      default: break;
    }
  }
};

#endif  // SIL_DYNAMIXEL2ARDUINO_H_
