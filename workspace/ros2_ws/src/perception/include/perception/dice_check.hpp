// 선반 위 3×3 주사위 확인 (ROS 의존 없음 → 단위 시험)
//   요청: 칸 중심 9곳(같은 좌표계, 판단이 계산해 줌)과 칸마다 기대값, 노랑 주사위 모양, 허용 오차
//   입력: 그 선반에서 본 퍽 목록(위치 · 색 · 모양). 선반 밖 퍽은 부르는 쪽이 거른다
//   1) 퍽마다 가장 가까운 칸에 넣는다
//   2) 칸마다: 퍽 수(0 · 1 · 2 이상), 중심 거리, 색(빨강 · 초록 칸) 또는 모양(노랑 칸)을 기대값과 비교
//   3) 모든 칸이 맞으면 통과. 틀린 칸은 번호와 이유를 돌려준다
//   칸 번호: 0 ~ 8, 행 우선 ((1,1) = 0, (1,3) = 2, (3,3) = 8)
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace perception
{

// 색 · 모양 값은 robot_interfaces/Puck 과 같다
enum PuckColor : uint8_t {COLOR_UNKNOWN = 0, COLOR_RED = 1, COLOR_GREEN = 2};
enum PuckShape : uint8_t {SHAPE_UNKNOWN = 0, SHAPE_CYLINDER = 1, SHAPE_CUBOID = 2};

struct SeenPuck
{
  double x = 0, y = 0;
  uint8_t color = COLOR_UNKNOWN;
  uint8_t shape = SHAPE_UNKNOWN;
};

struct DiceCell
{
  enum Expect : uint8_t {EMPTY = 0, RED = 1, GREEN = 2, YELLOW = 3};   // YELLOW: 색 무관, 모양 = yellow_shape
  double x = 0, y = 0;
  uint8_t expect = EMPTY;
};

struct DiceRequest
{
  std::array<DiceCell, 9> cells{};
  uint8_t yellow_shape = SHAPE_CYLINDER;   // 노랑 주사위에 쓰는 모양 (우리가 고름)
  double tolerance = 0.015;                // 칸 중심에서 벗어나도 되는 거리 [m] ★
};

struct CellResult
{
  enum Error : uint8_t {
    OK = 0, MISSING = 1, UNEXPECTED = 2, MULTIPLE = 3, OFFSET = 4, WRONG_COLOR = 5, WRONG_SHAPE = 6
  };
  uint8_t error = OK;
  int count = 0;         // 이 칸에 넣은 퍽 수
  double offset = 0;     // 가장 가까운 퍽의 중심 거리 [m] (퍽 없으면 0)
  uint8_t color = COLOR_UNKNOWN, shape = SHAPE_UNKNOWN;
};

struct DiceResult
{
  bool passed = false;
  std::array<CellResult, 9> cells{};
  std::vector<int> wrong;   // 틀린 칸 번호 (작은 순)
};

inline DiceResult check_dice(const DiceRequest & req, const std::vector<SeenPuck> & pucks)
{
  DiceResult r;
  std::array<std::vector<const SeenPuck *>, 9> in{};
  std::array<double, 9> best{};
  best.fill(std::numeric_limits<double>::infinity());
  for (const auto & p : pucks) {
    int k = 0;
    double dk = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 9; ++i) {
      const double d = std::hypot(p.x - req.cells[i].x, p.y - req.cells[i].y);
      if (d < dk) {dk = d; k = i;}
    }
    in[k].push_back(&p);
    if (dk < best[k]) {
      best[k] = dk;
      r.cells[k].color = p.color;
      r.cells[k].shape = p.shape;
    }
  }
  for (int i = 0; i < 9; ++i) {
    auto & c = r.cells[i];
    const auto & want = req.cells[i];
    c.count = static_cast<int>(in[i].size());
    c.offset = c.count ? best[i] : 0;
    if (want.expect == DiceCell::EMPTY) {
      c.error = c.count ? CellResult::UNEXPECTED : CellResult::OK;
    } else if (c.count == 0) {
      c.error = CellResult::MISSING;
    } else if (c.count > 1) {
      c.error = CellResult::MULTIPLE;
    } else if (c.offset > req.tolerance) {
      c.error = CellResult::OFFSET;
    } else if (want.expect == DiceCell::YELLOW) {
      c.error = c.shape == req.yellow_shape ? CellResult::OK : CellResult::WRONG_SHAPE;
    } else {
      const uint8_t color = want.expect == DiceCell::RED ? COLOR_RED : COLOR_GREEN;
      c.error = c.color == color ? CellResult::OK : CellResult::WRONG_COLOR;
    }
    if (c.error != CellResult::OK) {r.wrong.push_back(i);}
  }
  r.passed = r.wrong.empty();
  return r;
}

}  // namespace perception
