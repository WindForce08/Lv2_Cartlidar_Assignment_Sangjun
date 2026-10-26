// numpy 계산 재현 (ROS 의존 없음 → 단위 시험)
//   바코드 찾기(barcode_locator.hpp)는 기준 Python(locate_v2.py, numpy 1.26)과 결과가 같아야 한다(판정 시험을 그 코드로 했다).
//   문턱 바로 옆 값이 갈리지 않도록 numpy 가 다르게 계산하는 곳만 그대로 옮겼다.
//   - NumpyRng: np.random.default_rng(seed) = PCG64(SeedSequence(seed)), Generator.choice(n, k, replace=False)
//     (평면 나누기 RANSAC 이 같은 점을 뽑아야 함. bit_generator.pyx · pcg64.c · distributions.c · _generator.pyx)
//   - np_cosf · np_sinf: float32 cos · sin 은 numpy 가 SIMD 근사식을 써서 표준 cosf 와 1 ulp 다를 수 있다
//     (loops_trigonometric.dispatch.c.src: Cody-Waite 줄이기 + 다항식, FMA)
//   - np_sum_f32: float32 합은 8192 개씩 끊어 짝 합(pairwise)으로 더한다 → mean · std 도 같은 순서
//   - np_percentile · np_median: 선형 보간(_lerp) · 가운데 두 값 평균
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace perception
{

namespace npc
{
// float32 cos · sin (numpy SIMD 경로와 같은 값, |x| ≤ 71476 범위)
inline float np_trig_f32(float x, bool cosine)
{
  float q = x * 0x1.45f306p-1f;
  q = (q + 0x1.800000p+23f) - 0x1.800000p+23f;   // 가장 가까운 정수
  float r = std::fma(q, -0x1.921fb0p+00f, x);
  r = std::fma(q, -0x1.5110b4p-22f, r);
  r = std::fma(q, -0x1.846988p-48f, r);
  const float r2 = r * r;
  float c = std::fma(0x1.98e616p-16f, r2, -0x1.6c06dcp-10f);
  c = std::fma(c, r2, 0x1.55553cp-05f);
  c = std::fma(c, r2, -0x1.000000p-01f);
  c = std::fma(c, r2, 1.0f);
  float s = std::fma(0x1.7d3bbcp-19f, r2, -0x1.a06bbap-13f);
  s = std::fma(s, r2, 0x1.11119ap-07f);
  s = std::fma(s, r2, -0x1.555556p-03f);
  s = std::fma(s, r2, 0.0f);
  s = std::fma(s, r, r);
  int iq = static_cast<int>(std::nearbyint(q));
  if (cosine) {iq += 1;}
  float v = (iq & 1) == 0 ? s : c;
  if ((iq & 2) == 2) {v = 0.0f - v;}
  return v;
}
inline float np_cosf(float x) {return np_trig_f32(x, true);}
inline float np_sinf(float x) {return np_trig_f32(x, false);}
// np.radians(float32)
inline float np_radiansf(float deg) {return deg * static_cast<float>(M_PI / 180.0);}

template<class T>
T pairwise_sum(const T * a, size_t n)
{
  if (n < 8) {
    T r = 0;
    for (size_t i = 0; i < n; ++i) {r += a[i];}
    return r;
  }
  if (n <= 128) {
    T r[8];
    for (int j = 0; j < 8; ++j) {r[j] = a[j];}
    size_t i = 8;
    for (; i < n - (n % 8); i += 8) {
      for (int j = 0; j < 8; ++j) {r[j] += a[i + j];}
    }
    T res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
    for (; i < n; ++i) {res += a[i];}
    return res;
  }
  size_t n2 = n / 2;
  n2 -= n2 % 8;
  return pairwise_sum(a, n2) + pairwise_sum(a + n2, n - n2);
}

// np.sum(1 차원): 8192 개씩 끊어 짝 합
template<class T>
T np_sum(const std::vector<T> & a)
{
  T out = 0;
  for (size_t i = 0; i < a.size(); i += 8192) {out += pairwise_sum(a.data() + i, std::min<size_t>(8192, a.size() - i));}
  return out;
}

// np.std(float64)
inline double np_std_f64(const std::vector<double> & a)
{
  const double m = np_sum(a) / static_cast<double>(a.size());
  std::vector<double> d(a.size());
  for (size_t i = 0; i < a.size(); ++i) {d[i] = (a[i] - m) * (a[i] - m);}
  return std::sqrt(np_sum(d) / static_cast<double>(a.size()));
}

inline float pairwise_f32(const float * a, size_t n)
{
  if (n < 8) {
    float r = 0.0f;
    for (size_t i = 0; i < n; ++i) {r += a[i];}
    return r;
  }
  if (n <= 128) {
    float r[8];
    for (int j = 0; j < 8; ++j) {r[j] = a[j];}
    size_t i = 8;
    for (; i < n - (n % 8); i += 8) {
      for (int j = 0; j < 8; ++j) {r[j] += a[i + j];}
    }
    float res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
    for (; i < n; ++i) {res += a[i];}
    return res;
  }
  size_t n2 = n / 2;
  n2 -= n2 % 8;
  return pairwise_f32(a, n2) + pairwise_f32(a + n2, n - n2);
}

// np.sum(float32 1 차원)
inline float np_sum_f32(const std::vector<float> & a)
{
  float out = 0.0f;
  for (size_t i = 0; i < a.size(); i += 8192) {out += pairwise_f32(a.data() + i, std::min<size_t>(8192, a.size() - i));}
  return out;
}
// np.mean(float32)
inline float np_mean_f32(const std::vector<float> & a)
{
  return np_sum_f32(a) / static_cast<float>(a.size());
}
// np.std(float32): 평균 → 편차 제곱 → 합 / n → 제곱근, 모두 float32
inline float np_std_f32(const std::vector<float> & a)
{
  const float m = np_mean_f32(a);
  std::vector<float> d(a.size());
  for (size_t i = 0; i < a.size(); ++i) {
    const float x = a[i] - m;
    d[i] = x * x;
  }
  return std::sqrt(np_sum_f32(d) / static_cast<float>(a.size()));
}

// np.percentile(a, q) (method linear). 결과는 float64
template<class T>
double np_percentile(std::vector<T> a, double q)
{
  if (a.empty()) {return NAN;}
  const double vi = static_cast<double>(a.size() - 1) * (q / 100.0);
  const auto lo = static_cast<size_t>(std::floor(vi));
  const size_t hi = std::min(lo + 1, a.size() - 1);
  std::nth_element(a.begin(), a.begin() + static_cast<long>(lo), a.end());
  const double va = a[lo];
  const double vb = hi == lo ? va : static_cast<double>(*std::min_element(a.begin() + static_cast<long>(lo) + 1, a.end()));
  const double g = vi - static_cast<double>(lo);
  const double diff = vb - va;
  return g >= 0.5 ? vb - diff * (1.0 - g) : va + diff * g;
}

// np.median: 짝수 개면 가운데 두 값의 평균(입력 형식으로 계산)
template<class T>
T np_median(std::vector<T> a)
{
  if (a.empty()) {return static_cast<T>(NAN);}
  const size_t h = a.size() / 2;
  std::nth_element(a.begin(), a.begin() + static_cast<long>(h), a.end());
  const T m = a[h];
  if (a.size() % 2) {return m;}
  const T l = *std::max_element(a.begin(), a.begin() + static_cast<long>(h));
  return (l + m) / static_cast<T>(2);
}
}  // namespace npc

class NumpyRng
{
public:
  explicit NumpyRng(uint64_t seed)
  {
    // SeedSequence(seed): 엔트로피를 32 bit 낱말로 나눔 (0 이면 [0])
    std::vector<uint32_t> ent;
    do {ent.push_back(static_cast<uint32_t>(seed & 0xffffffffu)); seed >>= 32;} while (seed);
    uint32_t pool[4];
    uint32_t hc = INIT_A;
    auto hashmix = [&hc](uint32_t v) {
        v ^= hc;
        hc *= MULT_A;
        v *= hc;
        v ^= v >> 16;
        return v;
      };
    auto mix = [](uint32_t x, uint32_t y) {
        uint32_t r = MIX_L * x - MIX_R * y;
        r ^= r >> 16;
        return r;
      };
    for (size_t i = 0; i < 4; ++i) {pool[i] = hashmix(i < ent.size() ? ent[i] : 0u);}
    for (size_t s = 0; s < 4; ++s) {
      for (size_t d = 0; d < 4; ++d) {
        if (s != d) {pool[d] = mix(pool[d], hashmix(pool[s]));}
      }
    }
    for (size_t s = 4; s < ent.size(); ++s) {
      for (size_t d = 0; d < 4; ++d) {pool[d] = mix(pool[d], hashmix(ent[s]));}
    }
    // generate_state(4, uint64) = 32 bit 8 개 → 64 bit 4 개 (little endian)
    uint32_t st[8];
    uint32_t hb = INIT_B;
    for (int i = 0; i < 8; ++i) {
      uint32_t v = pool[i % 4];
      v ^= hb;
      hb *= MULT_B;
      v *= hb;
      v ^= v >> 16;
      st[i] = v;
    }
    uint64_t w[4];
    for (int i = 0; i < 4; ++i) {w[i] = static_cast<uint64_t>(st[2 * i]) | (static_cast<uint64_t>(st[2 * i + 1]) << 32);}
    // pcg64_set_seed: 상태 = (w0 << 64) + w1, 증분 = (w2 << 64) + w3
    const u128 init = (static_cast<u128>(w[0]) << 64) | w[1];
    const u128 seq = (static_cast<u128>(w[2]) << 64) | w[3];
    state_ = 0;
    inc_ = (seq << 1) | 1u;
    step();
    state_ += init;
    step();
  }

  uint64_t next_uint64()
  {
    step();
    const uint64_t x = static_cast<uint64_t>(state_ >> 64) ^ static_cast<uint64_t>(state_);
    const unsigned rot = static_cast<unsigned>(state_ >> 122);
    return (x >> rot) | (x << ((64u - rot) & 63u));
  }

  uint32_t next_uint32()
  {
    if (has32_) {has32_ = false; return buf32_;}
    const uint64_t n = next_uint64();
    has32_ = true;
    buf32_ = static_cast<uint32_t>(n >> 32);
    return static_cast<uint32_t>(n & 0xffffffffu);
  }

  // [0, rng] 안 정수 (random_bounded_uint64, use_masked = false). rng < 2^32 만 (화소 수)
  uint64_t bounded(uint64_t rng)
  {
    if (rng == 0) {return 0;}
    if (rng == 0xffffffffu) {return next_uint32();}
    const uint32_t excl = static_cast<uint32_t>(rng) + 1u;
    uint64_t m = static_cast<uint64_t>(next_uint32()) * excl;
    uint32_t left = static_cast<uint32_t>(m & 0xffffffffu);
    if (left < excl) {
      const uint32_t thr = (UINT32_MAX - static_cast<uint32_t>(rng)) % excl;
      while (left < thr) {
        m = static_cast<uint64_t>(next_uint32()) * excl;
        left = static_cast<uint32_t>(m & 0xffffffffu);
      }
    }
    return m >> 32;
  }

  // Generator.choice(pop, size, replace=False, shuffle=True): Floyd 방법 + 섞기 (pop ≤ 10000 이거나 size ≤ pop / 50 일 때 numpy 가 쓰는 길)
  std::vector<int64_t> choice(int64_t pop, int64_t size)
  {
    std::vector<int64_t> idx(static_cast<size_t>(size));
    uint64_t mask = static_cast<uint64_t>(1.2 * static_cast<double>(size));
    for (int sh = 1; sh < 64; sh <<= 1) {mask |= mask >> sh;}   // _gen_mask: 그 값 이상 가장 작은 2^k − 1
    std::vector<uint64_t> set(mask + 1, UINT64_MAX);
    for (int64_t j = pop - size; j < pop; ++j) {
      const uint64_t val = bounded(static_cast<uint64_t>(j));
      uint64_t loc = val & mask;
      while (set[loc] != UINT64_MAX && set[loc] != val) {loc = (loc + 1) & mask;}
      if (set[loc] == UINT64_MAX) {
        set[loc] = val;
        idx[static_cast<size_t>(j - pop + size)] = static_cast<int64_t>(val);
      } else {
        loc = static_cast<uint64_t>(j) & mask;
        while (set[loc] != UINT64_MAX) {loc = (loc + 1) & mask;}
        set[loc] = static_cast<uint64_t>(j);
        idx[static_cast<size_t>(j - pop + size)] = j;
      }
    }
    for (int64_t i = size - 1; i >= 1; --i) {   // _shuffle_int(size, 1, ...)
      const auto j = static_cast<size_t>(bounded(static_cast<uint64_t>(i)));
      std::swap(idx[static_cast<size_t>(i)], idx[j]);
    }
    return idx;
  }

private:
  __extension__ typedef unsigned __int128 u128;
  static constexpr uint32_t INIT_A = 0x43b0d7e5u, MULT_A = 0x931e8875u, INIT_B = 0x8b51f9ddu, MULT_B = 0x58f38dedu;
  static constexpr uint32_t MIX_L = 0xca01f9ddu, MIX_R = 0x4973f715u;
  void step()
  {
    static const u128 MULT = (static_cast<u128>(0x2360ed051fc65da4ULL) << 64) | 0x4385df649fccf645ULL;
    state_ = state_ * MULT + inc_;
  }
  u128 state_ = 0, inc_ = 0;
  bool has32_ = false;
  uint32_t buf32_ = 0;
};

}  // namespace perception
