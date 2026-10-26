// 바코드 라벨 찾기 (barcode_locator.hpp). 기준 Python locate_v2.py(45차)와 같은 순서 · 같은 계산 형식
//   주석 'Py:' 는 대응하는 Python 식. numpy 가 float32 로 계산하는 곳은 float 로, float64 인 곳은 double 로 계산한다
//   (문턱 바로 옆 값이 갈리지 않게. numpy 의 합 순서 · sin/cos 는 numpy_compat.hpp)
#include "perception/barcode_locator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/numpy_compat.hpp"

namespace perception
{

namespace
{
using Comp = std::pair<std::vector<int>, std::vector<int>>;   // ys, xs (행 순서)

// np.cross (float32): c0 = a1·b2 − a2·b1 …
cv::Vec3f cross_f(const cv::Vec3f & a, const cv::Vec3f & b)
{
  const float p0 = a[1] * b[2], q0 = a[2] * b[1];
  const float p1 = a[2] * b[0], q1 = a[0] * b[2];
  const float p2 = a[0] * b[1], q2 = a[1] * b[0];
  return {p0 - q0, p1 - q1, p2 - q2};
}

cv::Vec3d cross_d(const cv::Vec3d & a, const cv::Vec3d & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot_d(const cv::Vec3d & a, const cv::Vec3d & b) {return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];}

cv::Vec3d to_d(const cv::Vec3f & v) {return {v[0], v[1], v[2]};}

// 이어진 덩어리 (8 이웃). 덩어리마다 화소를 행 순서로 (Py: np.nonzero(lab == i))
std::vector<Comp> components(const cv::Mat & mask, int min_area)
{
  cv::Mat lab, st, cent;
  const int cnt = cv::connectedComponentsWithStats(mask, lab, st, cent, 8, CV_32S);
  std::vector<Comp> all(static_cast<size_t>(std::max(cnt, 1)));
  std::vector<char> want(static_cast<size_t>(std::max(cnt, 1)), 0);
  for (int i = 1; i < cnt; ++i) {
    want[static_cast<size_t>(i)] = st.at<int>(i, cv::CC_STAT_AREA) >= min_area;
    if (want[static_cast<size_t>(i)]) {
      all[static_cast<size_t>(i)].first.reserve(static_cast<size_t>(st.at<int>(i, cv::CC_STAT_AREA)));
      all[static_cast<size_t>(i)].second.reserve(static_cast<size_t>(st.at<int>(i, cv::CC_STAT_AREA)));
    }
  }
  for (int y = 0; y < lab.rows; ++y) {
    const int * l = lab.ptr<int>(y);
    for (int x = 0; x < lab.cols; ++x) {
      if (l[x] > 0 && want[static_cast<size_t>(l[x])]) {
        all[static_cast<size_t>(l[x])].first.push_back(y);
        all[static_cast<size_t>(l[x])].second.push_back(x);
      }
    }
  }
  std::vector<Comp> out;
  for (int i = 1; i < cnt; ++i) {
    if (want[static_cast<size_t>(i)]) {out.push_back(std::move(all[static_cast<size_t>(i)]));}
  }
  return out;
}

// LAPACK: numpy 1.26 의 np.linalg 는 float32 입력도 double 로 바꿔 아래 함수를 부르고 결과를 입력 형식으로 되돌린다
//   eigh → dsyevd('V', 'L'), eigvalsh → dsyevd('N', 'L'), inv → dgesv(B = I), lstsq → dgelsd(rcond = eps · max(M, N))
//   (같은 LAPACK 을 같은 인자로 부르면 같은 값 — ctypes 로 numpy 와 비트 단위로 같음을 확인)
extern "C" {
void dsyevd_(const char * jobz, const char * uplo, const int * n, double * a, const int * lda, double * w,
  double * work, const int * lwork, int * iwork, const int * liwork, int * info, size_t, size_t);
void dgesv_(const int * n, const int * nrhs, double * a, const int * lda, int * ipiv, double * b, const int * ldb, int * info);
void dgelsd_(const int * m, const int * n, const int * nrhs, double * a, const int * lda, double * b, const int * ldb,
  double * s, const double * rcond, int * rank, double * work, const int * lwork, int * iwork, int * info);
}

// 대칭 n × n (열 우선 a, 덮어씀) → 고유값 오름차순 w, jobz 'V' 면 a 의 열이 고유벡터
void lapack_syevd(int n, double * a, double * w, bool vectors)
{
  const char * jz = vectors ? "V" : "N";
  double wq = 0;
  int iq = 0, info = 0, lw = -1, liw = -1;
  dsyevd_(jz, "L", &n, a, &n, w, &wq, &lw, &iq, &liw, &info, 1, 1);   // 작업 공간 묻기 (numpy 와 같게)
  lw = static_cast<int>(wq);
  liw = std::max(iq, 1);
  std::vector<double> work(static_cast<size_t>(std::max(lw, 1)));
  std::vector<int> iwork(static_cast<size_t>(liw));
  dsyevd_(jz, "L", &n, a, &n, w, work.data(), &lw, iwork.data(), &liw, &info, 1, 1);
}

// Py: np.linalg.inv (2 × 2)
cv::Matx22d lapack_inv2(const cv::Matx22d & M)
{
  double a[4] = {M(0, 0), M(1, 0), M(0, 1), M(1, 1)}, b[4] = {1, 0, 0, 1};
  int n = 2, info = 0, ip[2];
  dgesv_(&n, &n, a, &n, ip, b, &n, &info);
  return cv::Matx22d(b[0], b[2], b[1], b[3]);
}

// Py: np.linalg.eigvalsh(S).max() (2 × 2)
double lapack_eigmax2(const cv::Matx22d & S)
{
  double a[4] = {S(0, 0), S(1, 0), S(0, 1), S(1, 1)}, w[2];
  lapack_syevd(2, a, w, false);
  return w[1];
}

// Py: np.linalg.lstsq(A, y, rcond=None)[0]. A 는 m × n 열 우선
std::vector<double> lapack_lstsq(int m, int n, std::vector<double> A, const std::vector<double> & y)
{
  const int ldb = std::max(m, n), one = 1;
  std::vector<double> b(static_cast<size_t>(ldb), 0.0), sv(static_cast<size_t>(std::min(m, n)));
  std::copy(y.begin(), y.end(), b.begin());
  const double rcond = std::numeric_limits<double>::epsilon() * std::max(m, n);
  int rank = 0, info = 0, lw = -1, iq = 0;
  double wq = 0;
  dgelsd_(&m, &n, &one, A.data(), &m, b.data(), &ldb, sv.data(), &rcond, &rank, &wq, &lw, &iq, &info);
  lw = static_cast<int>(wq);
  std::vector<double> work(static_cast<size_t>(std::max(lw, 1)));
  std::vector<int> iwork(static_cast<size_t>(std::max(iq, 1)));
  dgelsd_(&m, &n, &one, A.data(), &m, b.data(), &ldb, sv.data(), &rcond, &rank, work.data(), &lw, iwork.data(), &info);
  b.resize(static_cast<size_t>(n));
  return b;
}

// 평면 맞춤 — numpy 와 같은 계산 (numpy 1.26 + 표준 netlib BLAS · LAPACK, 이 PC 의 python3-numpy 구성):
//   c0 = Q.mean(0): 열마다 float32 차례 합 / N
//   C = (Q − c0)ᵀ(Q − c0) / N: cblas_sgemm → 표준 sgemm 의 합 순서(화소 차례, float32)
//   w, V = eigh(C): double 로 dsyevd 뒤 float32 로. 첫 열이 가장 작은 고유값의 벡터
//   (C 를 double 로 구하면 좁은 평면에서 법선이 1e-4 까지 달라져 상관 점수가 문턱 양쪽으로 갈림 — R3 렌더 비교에서 확인)
struct Fit
{
  cv::Vec3f c0;
  float w0 = 0;              // 가장 작은 고유값
  cv::Vec3f v0;              // 그 고유벡터
};

Fit fit_plane(const cv::Mat & P, const std::vector<int> & ys, const std::vector<int> & xs)
{
  Fit f;
  const size_t N = ys.size();
  float s[3] = {0.f, 0.f, 0.f};
  for (size_t i = 0; i < N; ++i) {
    const cv::Vec3f & q = P.at<cv::Vec3f>(ys[i], xs[i]);
    s[0] += q[0];
    s[1] += q[1];
    s[2] += q[2];
  }
  const float nf = static_cast<float>(N);
  f.c0 = {s[0] / nf, s[1] / nf, s[2] / nf};
  float C[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};   // 대칭
  for (size_t i = 0; i < N; ++i) {
    const cv::Vec3f & q = P.at<cv::Vec3f>(ys[i], xs[i]);
    const float d[3] = {q[0] - f.c0[0], q[1] - f.c0[1], q[2] - f.c0[2]};
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {C[a * 3 + b] += d[b] * d[a];}
    }
  }
  for (auto & v : C) {v /= nf;}
  double A[9], w[3];
  for (int i = 0; i < 9; ++i) {A[i] = C[i];}   // 대칭이라 행 · 열 우선 같음
  lapack_syevd(3, A, w, true);
  f.w0 = static_cast<float>(w[0]);
  f.v0 = {static_cast<float>(A[0]), static_cast<float>(A[1]), static_cast<float>(A[2])};   // 첫 열
  return f;
}

// Py: _ncc(w, t, m) — 모두 float32: 평균 빼기(짝 합 평균), 곱의 합은 cblas_sdot(표준 BLAS 는 차례 합)
double ncc(const std::vector<float> & w_in, const std::vector<float> & t_in)
{
  const float mw = npc::np_mean_f32(w_in), mt = npc::np_mean_f32(t_in);
  float wt = 0, ww = 0, tt = 0;
  for (size_t i = 0; i < w_in.size(); ++i) {wt += (w_in[i] - mw) * (t_in[i] - mt);}
  for (size_t i = 0; i < w_in.size(); ++i) {ww += (w_in[i] - mw) * (w_in[i] - mw);}
  for (size_t i = 0; i < w_in.size(); ++i) {tt += (t_in[i] - mt) * (t_in[i] - mt);}
  return wt / std::max(std::sqrt(ww * tt), 1e-9f);
}

// Py: k /= k.sum() (float32 나눗셈. OpenCV 의 Mat /= s 는 1/s 곱이라 값이 조금 다름)
void div_by_sum(cv::Mat & k)
{
  std::vector<float> kv(k.begin<float>(), k.end<float>());
  const float s = npc::np_sum_f32(kv);
  for (auto it = k.begin<float>(); it != k.end<float>(); ++it) {*it = *it / s;}
}

// mask(CV_8U) 가 0 이 아닌 자리의 값 (행 순서, Py: a[m])
std::vector<float> masked(const cv::Mat & a, const cv::Mat & m)
{
  std::vector<float> out;
  for (int y = 0; y < a.rows; ++y) {
    const float * p = a.ptr<float>(y);
    const uchar * q = m.ptr<uchar>(y);
    for (int x = 0; x < a.cols; ++x) {
      if (q[x]) {out.push_back(p[x]);}
    }
  }
  return out;
}

struct ShapeFeats
{
  double resw_lo = 0, side_lo = 0, sharp = 0, fit_a = 0;
};

// Py: _shape_feats — 후보 창을 '밝기 a · 흐린 템플릿 + b' 로 맞춘 뒤 남는 것과, 더 흐린 템플릿과 견준 선명도
ShapeFeats shape_feats(const cv::Mat & win, const cv::Mat & tpb, const cv::Mat & tp0, const cv::Mat & mk0, const cv::Mat & ker,
  const cv::Mat & bright, double th_deg, double lw, double lh)
{
  ShapeFeats f;
  cv::Mat m = mk0 > 0.5f;
  const std::vector<float> tv = masked(tpb, m), wv = masked(win, m);
  // 최소제곱 [t 1]·[a b] = w (Py: np.linalg.lstsq, float32 입력 → 결과 float32)
  const int nm = static_cast<int>(tv.size());
  std::vector<double> A(2 * tv.size()), y(wv.begin(), wv.end());
  for (size_t i = 0; i < tv.size(); ++i) {A[i] = tv[i]; A[tv.size() + i] = 1.0;}
  const auto ab = lapack_lstsq(nm, 2, A, y);
  const float a = static_cast<float>(ab[0]), b = static_cast<float>(ab[1]);
  const float tmax = *std::max_element(tv.begin(), tv.end()), tmin = *std::min_element(tv.begin(), tv.end());
  const float span = std::max(std::fabs(a) * (tmax - tmin), 1e-6f);   // 템플릿이 예상하는 명암 폭
  cv::Mat R_(win.size(), CV_32F);
  for (int y = 0; y < win.rows; ++y) {
    for (int x = 0; x < win.cols; ++x) {
      R_.at<float>(y, x) = (win.at<float>(y, x) - (a * tpb.at<float>(y, x) + b)) / span;
    }
  }
  // 흐림을 두 배로 한 템플릿
  cv::Mat k2;
  cv::resize(ker, k2, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
  div_by_sum(k2);
  cv::Mat tb2;
  cv::filter2D(tp0, tb2, -1, k2, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  std::vector<float> rw;
  if (cv::countNonZero(bright) > 10) {
    rw = masked(R_, bright);
  } else {
    rw = masked(R_, m);
  }
  // 흰 테두리를 네 변(위 · 아래 · 왼 · 오른)으로 나눠 가장 어두운 변의 평균 잔차
  {
    const int D = win.rows;
    const double th = th_deg * M_PI / 180.0, ct = std::cos(th), sn = std::sin(th);
    const double hw = lw / 2, hh = lh / 2;
    std::array<std::vector<float>, 4> sides;
    for (int y = 0; y < D; ++y) {
      for (int x = 0; x < D; ++x) {
        if (!bright.at<uchar>(y, x)) {continue;}
        const double dx = x - (D - 1) / 2.0, dy = y - (D - 1) / 2.0;
        const double la = dx * ct - dy * sn, lb = dx * sn + dy * ct;
        const float r = R_.at<float>(y, x);
        if (lb < -hh * 0.5) {sides[0].push_back(r);}
        if (lb > hh * 0.5) {sides[1].push_back(r);}
        if (la < -hw * 0.6) {sides[2].push_back(r);}
        if (la > hw * 0.6) {sides[3].push_back(r);}
      }
    }
    double lo = std::numeric_limits<double>::infinity();
    for (const auto & s : sides) {
      if (s.size() >= 5) {lo = std::min(lo, static_cast<double>(npc::np_mean_f32(s)));}
    }
    f.side_lo = lo;
  }
  f.resw_lo = npc::np_percentile(rw, 2.0);
  f.sharp = ncc(wv, tv) - ncc(wv, masked(tb2, m));
  f.fit_a = a;
  return f;
}
}  // namespace

// ------------------------------ 템플릿 ------------------------------
BarcodeLocator::BarcodeLocator(const BarcodeLocatorConfig & cfg, const std::string & tex_path)
: cfg_(cfg)
{
  const cv::Mat tex = cv::imread(tex_path, cv::IMREAD_GRAYSCALE);
  if (!tex.empty()) {build_templates(tex);}
}

BarcodeLocator::BarcodeLocator(const BarcodeLocatorConfig & cfg, const cv::Mat & tex_gray)
: cfg_(cfg)
{
  if (!tex_gray.empty()) {build_templates(tex_gray);}
}

void BarcodeLocator::build_templates(const cv::Mat & tex_gray)
{
  const auto & c = cfg_;
  if (c.mm <= 0 || c.rot_step <= 0) {return;}
  cv::Mat tex;
  tex_gray.convertTo(tex, CV_32F);
  const int w = static_cast<int>(std::nearbyint(c.label_w_mm / c.mm)), h = static_cast<int>(std::nearbyint(c.label_h_mm / c.mm));
  if (w < 2 || h < 2) {return;}
  cv::Mat t;
  cv::resize(tex, t, cv::Size(w, h), 0, 0, cv::INTER_AREA);
  const int D = static_cast<int>(std::ceil(std::hypot(w, h))) | 1;
  const cv::Mat ones = cv::Mat::ones(h, w, CV_32F);
  cv::Mat keep = cv::Mat::zeros(h, w, CV_32F);
  keep.rowRange(0, std::min(h, static_cast<int>(std::nearbyint(c.text_cut_mm / c.mm)))).setTo(1.0f);
  cv::Mat white_src = cv::Mat::zeros(h, w, CV_32F), dark_src = cv::Mat::zeros(h, w, CV_32F);
  white_src.setTo(1.0f, t >= c.tpl_white);
  dark_src.setTo(1.0f, t <= c.tpl_dark);
  const int n = static_cast<int>(std::ceil(360.0 / c.rot_step));   // Py: np.arange(0, 360, rot_step)
  for (int i = 0; i < n; ++i) {
    Template T;
    T.theta = i * c.rot_step;
    cv::Mat M = cv::getRotationMatrix2D(cv::Point2f((w - 1) / 2.0f, (h - 1) / 2.0f), T.theta, 1.0);
    M.at<double>(0, 2) += (D - w) / 2.0;
    M.at<double>(1, 2) += (D - h) / 2.0;
    // 바깥은 가장자리 값으로 채움: 나중에 화소 넓이만큼 흐릴 때 0 이 흰 테두리로 번지지 않게
    cv::warpAffine(t, T.tp, M, cv::Size(D, D), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    auto rot = [&](const cv::Mat & a) {
        cv::Mat r;
        cv::warpAffine(a, r, M, cv::Size(D, D), cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
        return cv::Mat(r > 0.5f);
      };
    cv::Mat mk = rot(ones);
    if (!c.use_text) {mk &= rot(keep);}   // 글자 줄(아래 4.5 mm)은 실물 라벨마다 다를 수 있어 뺄 수 있게
    T.mk = cv::Mat::zeros(D, D, CV_32F);
    T.mk.setTo(1.0f, mk);
    T.bright = rot(white_src) & mk;
    T.dark = rot(dark_src) & mk;
    T.bright /= 255;
    T.dark /= 255;
    tpls_.push_back(T);
  }
}

// ------------------------------ 평면 ------------------------------
void BarcodeLocator::split_component(const cv::Mat & P, std::vector<int> ys, std::vector<int> xs, int depth,
  std::vector<Comp> & out) const
{
  // 평면 하나로 맞지 않는 덩어리(잡음으로 모서리 너머 면과 이어진 것)를 RANSAC 으로 평면 하나씩 떼어 낸다
  const auto & c = cfg_;
  const Fit f = fit_plane(P, ys, xs);
  const float rms0 = f.w0 > 0 ? std::sqrt(f.w0) : 0.0f;   // Py: np.sqrt(max(w_[0], 0)) (float32)
  if (rms0 <= static_cast<float>(c.split_tol / 2) || depth >= 3) {
    out.emplace_back(std::move(ys), std::move(xs));
    return;
  }
  const size_t N = ys.size();
  std::vector<cv::Vec3f> Q(N);
  for (size_t i = 0; i < N; ++i) {Q[i] = P.at<cv::Vec3f>(ys[i], xs[i]);}
  NumpyRng rng(static_cast<uint64_t>(N));
  const float tol = static_cast<float>(c.split_tol);
  std::vector<char> best, inl(N);
  size_t bn = 0;
  for (int it = 0; it < c.split_iter; ++it) {
    const auto idx = rng.choice(static_cast<int64_t>(N), 3);
    const cv::Vec3f & a = Q[static_cast<size_t>(idx[0])], & b = Q[static_cast<size_t>(idx[1])], & cc = Q[static_cast<size_t>(idx[2])];
    const cv::Vec3f n = cross_f(b - a, cc - a);
    const float nn = std::sqrt((n[0] * n[0] + n[1] * n[1]) + n[2] * n[2]);
    if (nn < 1e-9f) {continue;}
    const cv::Vec3f u(n[0] / nn, n[1] / nn, n[2] / nn);
    size_t cnt = 0;
    for (size_t i = 0; i < N; ++i) {
      const float d = (Q[i][0] - a[0]) * u[0] + (Q[i][1] - a[1]) * u[1] + (Q[i][2] - a[2]) * u[2];
      inl[i] = std::fabs(d) < tol;
      cnt += static_cast<size_t>(inl[i]);
    }
    if (cnt > bn) {best = inl; bn = cnt;}
  }
  if (best.empty() || bn < static_cast<size_t>(c.min_comp)) {
    out.emplace_back(std::move(ys), std::move(xs));
    return;
  }
  for (int pass = 0; pass < 2; ++pass) {   // 맞는 쪽 · 나머지 각각 다시 이어진 덩어리로 나눔
    cv::Mat m = cv::Mat::zeros(P.rows, P.cols, CV_8U);
    for (size_t i = 0; i < N; ++i) {
      if (static_cast<bool>(best[i]) == (pass == 0)) {m.at<uchar>(ys[i], xs[i]) = 1;}
    }
    for (auto & comp : components(m, c.min_comp)) {
      if (pass == 0) {
        out.push_back(std::move(comp));
      } else {
        split_component(P, std::move(comp.first), std::move(comp.second), depth + 1, out);
      }
    }
  }
}

std::vector<BarcodePlane> BarcodeLocator::surfaces(const cv::Mat & dep_in, const BarcodeK & K, const cv::Matx44d & T,
  cv::Mat * P_out, cv::Mat * zf_out) const
{
  // 화소마다 법선(base 기준)을 구해 벽(법선 수평) · 윗면(법선 수직) 평면 덩어리로 나눈다
  const auto & c = cfg_;
  cv::Mat dep;
  dep_in.convertTo(dep, CV_32F);
  const int H = dep.rows, W = dep.cols;
  cv::Mat ok(H, W, CV_8U), z(H, W, CV_32F);
  const float dmin = static_cast<float>(c.depth_min), dmax = static_cast<float>(c.depth_max);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const float v = dep.at<float>(y, x);
      const bool o = std::isfinite(v) && v > dmin && v < dmax;
      ok.at<uchar>(y, x) = o;
      z.at<float>(y, x) = o ? v : 0.0f;
    }
  }
  if (c.median) {   // 깊이 잡음(실물 D435 는 1 m 에서 수 mm) 줄이기
    cv::Mat mz;
    cv::medianBlur(z, mz, c.median);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {z.at<float>(y, x) = ok.at<uchar>(y, x) ? mz.at<float>(y, x) : 0.0f;}
    }
  }
  const float fx = static_cast<float>(K.fx), fy = static_cast<float>(K.fy), cx = static_cast<float>(K.cx), cy = static_cast<float>(K.cy);
  cv::Mat P(H, W, CV_32FC3), Pc[3];
  for (auto & m : Pc) {m.create(H, W, CV_32F);}
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const float zz = z.at<float>(y, x);
      const cv::Vec3f p((static_cast<float>(x) - cx) / fx * zz, (static_cast<float>(y) - cy) / fy * zz, zz);
      P.at<cv::Vec3f>(y, x) = p;
      for (int i = 0; i < 3; ++i) {Pc[i].at<float>(y, x) = p[i];}
    }
  }
  const int s = c.smooth;
  cv::Mat okf, wsum, Ps[3];
  ok.convertTo(okf, CV_32F);
  cv::blur(okf, wsum, cv::Size(s, s));
  for (int i = 0; i < 3; ++i) {
    cv::blur(Pc[i], Ps[i], cv::Size(s, s));
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {Ps[i].at<float>(y, x) /= std::max(wsum.at<float>(y, x), 1e-6f);}
    }
  }
  const int k = c.nk;
  cv::Mat n(H, W, CV_32FC3);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      cv::Vec3f dx(0, 0, 0), dy(0, 0, 0), ps;
      for (int i = 0; i < 3; ++i) {
        ps[i] = Ps[i].at<float>(y, x);
        if (x >= k && x < W - k) {dx[i] = Ps[i].at<float>(y, x + k) - Ps[i].at<float>(y, x - k);}
        if (y >= k && y < H - k) {dy[i] = Ps[i].at<float>(y + k, x) - Ps[i].at<float>(y - k, x);}
      }
      cv::Vec3f v = cross_f(dx, dy);
      const float nr = std::max(std::sqrt((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]), 1e-9f);
      v = cv::Vec3f(v[0] / nr, v[1] / nr, v[2] / nr);
      const float sd = ((v[0] * ps[0] + v[1] * ps[1]) + v[2] * ps[2]) + 1e-12f;   // 카메라 쪽을 보게
      const float sg = -static_cast<float>((sd > 0) - (sd < 0));
      n.at<cv::Vec3f>(y, x) = cv::Vec3f(v[0] * sg, v[1] * sg, v[2] * sg);
    }
  }
  const cv::Matx33d R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
  const cv::Vec3d t(T(0, 3), T(1, 3), T(2, 3));
  const float ce = static_cast<float>(std::cos(c.edge_deg * M_PI / 180.0));
  const float dzr = static_cast<float>(c.edge_dz_ratio), dzf = static_cast<float>(c.edge_dz_floor);
  cv::Mat wall_m = cv::Mat::zeros(H, W, CV_8U), top_m = cv::Mat::zeros(H, W, CV_8U);
  cv::Mat Pbx(H, W, CV_64F), Pby(H, W, CV_64F), Pbz(H, W, CV_64F);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const cv::Vec3f & v = n.at<cv::Vec3f>(y, x);
      const cv::Vec3f & p = P.at<cv::Vec3f>(y, x);
      const double nbz = v[0] * R(2, 0) + v[1] * R(2, 1) + static_cast<double>(v[2]) * R(2, 2);
      double pb[3];
      for (int i = 0; i < 3; ++i) {pb[i] = p[0] * R(i, 0) + p[1] * R(i, 1) + static_cast<double>(p[2]) * R(i, 2) + t[i];}
      Pbx.at<double>(y, x) = pb[0];
      Pby.at<double>(y, x) = pb[1];
      Pbz.at<double>(y, x) = pb[2];
      // 경계: 이웃(오른쪽 k · 아래 k, Py: np.roll) 법선 각 차 · 깊이 차
      bool edge = false;
      const float zz = z.at<float>(y, x);
      const int xs_[2] = {((x - k) % W + W) % W, x}, ys_[2] = {y, ((y - k) % H + H) % H};
      for (int j = 0; j < 2; ++j) {
        const cv::Vec3f & v2 = n.at<cv::Vec3f>(ys_[j], xs_[j]);
        edge = edge || ((v[0] * v2[0] + v[1] * v2[1]) + v[2] * v2[2]) < ce;
        edge = edge || std::fabs(zz - z.at<float>(ys_[j], xs_[j])) > dzr * std::max(zz, dzf);
      }
      const bool good = ok.at<uchar>(y, x) && wsum.at<float>(y, x) > 0.99f && !edge && pb[2] >= c.min_z &&
        std::hypot(pb[0], pb[1]) <= c.rng;
      if (!good) {continue;}
      if (std::fabs(nbz) <= c.nz_wall) {wall_m.at<uchar>(y, x) = 1;}
      if (nbz >= c.nz_top) {top_m.at<uchar>(y, x) = 1;}
    }
  }
  if (c.top_h > 0) {
    // 좁은 틈(벽 · 퍽 사이 수십 mm)의 윗면은 법선 창에 옆 면이 섞여 법선이 틀어진다.
    // 수평면은 깊이 잡음(시선 방향)이 있어도 높이는 거의 그대로 → 작은 창에서 '높이 퍼짐 / 수평 퍼짐' 이 작으면 윗면으로 더한다
    // (벽 · 퍽 옆면은 수평 퍼짐이 작고 높이 퍼짐이 큼). 이 화소에는 법선 경계 조건을 쓰지 않는다
    const int hk = c.top_hk;
    cv::Mat wk;
    cv::blur(okf, wk, cv::Size(hk, hk));
    cv::Mat okd;
    okf.convertTo(okd, CV_64F);
    auto var = [&](const cv::Mat & a) {   // Py: blur(a·ok) / max(wk, 1e-6) … (a 는 float64)
        cv::Mat s1, s2;
        cv::blur(a.mul(okd), s1, cv::Size(hk, hk));
        cv::blur(a.mul(a).mul(okd), s2, cv::Size(hk, hk));
        cv::Mat v(a.size(), CV_64F);
        for (int y = 0; y < H; ++y) {
          for (int x = 0; x < W; ++x) {
            const double w = std::max(wk.at<float>(y, x), 1e-6f);
            const double m1 = s1.at<double>(y, x) / w;
            v.at<double>(y, x) = s2.at<double>(y, x) / w - m1 * m1;
          }
        }
        return v;
      };
    const cv::Mat vh = var(Pbz), vx = var(Pbx), vy = var(Pby);
    cv::Mat jump = cv::Mat::zeros(H, W, CV_8U);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const float zz = z.at<float>(y, x);
        const float lim = dzr * std::max(zz, dzf);
        const int nb[4][2] = {{y, (x - 1 + W) % W}, {(y - 1 + H) % H, x}, {y, (x + 1) % W}, {(y + 1) % H, x}};   // Py: np.roll
        bool j = false;
        for (const auto & q : nb) {j = j || std::fabs(zz - z.at<float>(q[0], q[1])) > lim;}
        jump.at<uchar>(y, x) = j;
      }
    }
    cv::Mat jd;
    cv::dilate(jump, jd, cv::Mat::ones(hk, hk, CV_8U));
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const double vxy = vx.at<double>(y, x) + vy.at<double>(y, x);
        const bool flat = std::sqrt(std::max(vh.at<double>(y, x), 0.0) / std::max(vxy, 1e-12)) <= c.top_h;
        if (ok.at<uchar>(y, x) && wk.at<float>(y, x) > 0.99f && flat && !jd.at<uchar>(y, x) && Pbz.at<double>(y, x) >= c.min_z &&
          std::hypot(Pbx.at<double>(y, x), Pby.at<double>(y, x)) <= c.rng)
        {
          top_m.at<uchar>(y, x) = 1;
        }
      }
    }
  }
  std::vector<BarcodePlane> planes;
  for (int kind = 0; kind < 2; ++kind) {
    std::vector<Comp> comps;
    for (auto & comp : components(kind == 0 ? wall_m : top_m, c.min_comp)) {
      if (c.split) {
        split_component(P, std::move(comp.first), std::move(comp.second), 0, comps);
      } else {
        comps.push_back(std::move(comp));
      }
    }
    for (auto & comp : comps) {
      BarcodePlane pl;
      pl.kind = kind == 0 ? BarcodeDetection::WALL : BarcodeDetection::TOP;
      const Fit f = fit_plane(P, comp.first, comp.second);
      const cv::Vec3d c0d = to_d(f.c0);
      const float vc = (f.v0[0] * f.c0[0] + f.v0[1] * f.c0[1]) + f.v0[2] * f.c0[2];
      cv::Vec3f nrm_f = vc > 0 ? cv::Vec3f(-f.v0[0], -f.v0[1], -f.v0[2]) : f.v0;
      cv::Vec3d nrm = to_d(nrm_f);
      double w0 = f.w0 > 0 ? static_cast<double>(std::sqrt(f.w0)) : 0.0;   // 아래에서 rms (Py: float(np.sqrt(max(w, 0))))
      bool constrained = false;
      const size_t N = comp.first.size();
      if (c.constrain == 1 || (c.constrain == 2 && kind == 1)) {
        // 윗면은 수평(법선 = base z축), 벽은 수직(법선이 수평): 작은 평면에서 잡음으로 법선이 틀어지지 않게 자유도를 줄임
        cv::Vec3d nb(0, 0, 1);
        if (kind == 0) {
          std::vector<cv::Vec2d> xy(N);
          cv::Vec2d m(0, 0);
          for (size_t i = 0; i < N; ++i) {
            const cv::Vec3f & p = P.at<cv::Vec3f>(comp.first[i], comp.second[i]);
            for (int r = 0; r < 2; ++r) {xy[i][r] = p[0] * R(r, 0) + p[1] * R(r, 1) + static_cast<double>(p[2]) * R(r, 2) + t[r];}
            m += xy[i];
          }
          m /= static_cast<double>(N);
          cv::Matx22d C = cv::Matx22d::zeros();
          for (auto & q : xy) {
            q -= m;
            C(0, 0) += q[0] * q[0]; C(0, 1) += q[0] * q[1]; C(1, 1) += q[1] * q[1];
          }
          C(1, 0) = C(0, 1);
          double A2[4] = {C(0, 0), C(1, 0), C(0, 1), C(1, 1)}, w2[2];
          lapack_syevd(2, A2, w2, true);
          nb = {A2[0], A2[1], 0.0};
        }
        nrm = R.t() * nb;
        nrm *= dot_d(nrm, c0d) > 0 ? -1.0 : 1.0;
        double ss = 0;
        for (size_t i = 0; i < N; ++i) {
          const cv::Vec3f & q = P.at<cv::Vec3f>(comp.first[i], comp.second[i]);
          const double r = (q[0] - f.c0[0]) * nrm[0] + (q[1] - f.c0[1]) * nrm[1] + static_cast<double>(q[2] - f.c0[2]) * nrm[2];
          ss += r * r;
        }
        w0 = std::sqrt(ss / static_cast<double>(N));
        nrm_f = cv::Vec3f(static_cast<float>(nrm[0]), static_cast<float>(nrm[1]), static_cast<float>(nrm[2]));
        constrained = true;
      }
      std::vector<float> r(N), zq(N);
      std::vector<double> pbz(N);
      for (size_t i = 0; i < N; ++i) {
        const cv::Vec3f & q = P.at<cv::Vec3f>(comp.first[i], comp.second[i]);
        if (constrained) {   // Py: float64 nrm
          r[i] = static_cast<float>((q[0] - f.c0[0]) * nrm[0] + (q[1] - f.c0[1]) * nrm[1] + static_cast<double>(q[2] - f.c0[2]) * nrm[2]);
        } else {             // Py: (Q − c0) @ nrm, float32 sgemv (차례 합)
          const float d0 = q[0] - f.c0[0], d1 = q[1] - f.c0[1], d2 = q[2] - f.c0[2];
          r[i] = (d0 * nrm_f[0] + d1 * nrm_f[1]) + d2 * nrm_f[2];
        }
        zq[i] = q[2];
        pbz[i] = Pbz.at<double>(comp.first[i], comp.second[i]);
      }
      const float rm = npc::np_median(r);
      for (auto & v : r) {v = std::fabs(v - rm);}
      pl.sig = 1.4826 * static_cast<double>(npc::np_median(r));   // 이 평면에서 잰 깊이 잡음(강건 표준편차)
      pl.c = f.c0;
      pl.n = nrm_f;
      pl.rms = w0;
      pl.zmed = npc::np_median(zq);
      pl.ztop = npc::np_percentile(pbz, 98.0);
      pl.ys = std::move(comp.first);
      pl.xs = std::move(comp.second);
      planes.push_back(std::move(pl));
    }
  }
  if (P_out) {*P_out = P;}
  if (zf_out) {*zf_out = z;}
  return planes;
}

// ------------------------------ 찾기 ------------------------------
std::vector<BarcodeDetection> BarcodeLocator::locate(const cv::Mat & bgr, const cv::Mat & depth, const BarcodeK & K,
  const cv::Matx44d & T) const
{
  std::vector<BarcodeDetection> out;
  if (!ok() || bgr.empty() || bgr.type() != CV_8UC3 || depth.size() != bgr.size()) {return out;}
  const auto & c = cfg_;
  cv::Mat g, sat, hsv, g8;
  cv::cvtColor(bgr, g8, cv::COLOR_BGR2GRAY);
  g8.convertTo(g, CV_32F);
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::extractChannel(hsv, g8, 1);
  g8.convertTo(sat, CV_32F);
  const int H = g.rows, W = g.cols;
  cv::Mat P, zf;
  const auto planes = surfaces(depth, K, T, &P, &zf);
  const cv::Matx33d R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
  const cv::Vec3d zb_cam(R(2, 0), R(2, 1), R(2, 2));   // base 위쪽 방향을 카메라 좌표로 (Rᵀ·z)
  const double mm = c.mm / 1000;
  const double lw = c.label_w_mm / c.mm, lh = c.label_h_mm / c.mm;
  auto proj = [&K](const cv::Vec3d & X) {return cv::Vec2d(K.fx * X[0] / X[2] + K.cx, K.fy * X[1] / X[2] + K.cy);};

  for (const auto & pl : planes) {
    if (pl.rms > c.plane_rms + 2 * pl.sig) {continue;}   // 잡음보다 크게 휜 면은 평면이 아님
    // 과제 규칙: 라벨은 벽판 또는 상자 윗면에만(상자 옆면 · 바닥은 안 됨) → 수직면은 상자보다 높이 올라가야 벽으로 봄
    if (pl.kind == BarcodeDetection::WALL && pl.ztop < c.wall_min_top) {continue;}
    const cv::Vec3d n = to_d(pl.n), c0 = to_d(pl.c);
    cv::Vec3d e1;
    if (pl.kind == BarcodeDetection::WALL) {
      e1 = cross_d(zb_cam, n);
    } else {
      e1 = cv::Vec3d(R(0, 0), R(0, 1), R(0, 2));
      e1 -= dot_d(e1, n) * n;
    }
    e1 /= std::sqrt(dot_d(e1, e1));
    const cv::Vec3d e2 = cross_d(e1, n);   // 카메라 쪽에서 본 오른쪽 · 아래 (거울상 방지)
    const size_t N = pl.ys.size();
    std::vector<double> sv(N), tv(N);
    for (size_t i = 0; i < N; ++i) {
      const cv::Vec3f & p = P.at<cv::Vec3f>(pl.ys[i], pl.xs[i]);
      const cv::Vec3d q(p[0] - pl.c[0], p[1] - pl.c[1], p[2] - pl.c[2]);   // float32 차
      sv[i] = dot_d(q, e1);
      tv[i] = dot_d(q, e2);
    }
    const double m = c.rect_margin;   // 법선 다듬기로 평면 가장자리가 깎이므로 둘레까지 펴서 깊이로 평면 위인지 본다
    const double s0 = *std::min_element(sv.begin(), sv.end()) - m, s1 = *std::max_element(sv.begin(), sv.end()) + m;
    const double t0 = *std::min_element(tv.begin(), tv.end()) - m, t1 = *std::max_element(tv.begin(), tv.end()) + m;
    // 평면 가로 폭(벽은 e1 이 수평): 라벨은 벽판(380 mm)에만 붙고, 미로 기둥 옆면(30 ~ 36 mm)에는 없다
    const double pl_w = npc::np_percentile(sv, 99.0) - npc::np_percentile(sv, 1.0);
    if (pl.kind == BarcodeDetection::WALL && pl_w < c.wall_min_w) {continue;}
    const int ns = static_cast<int>((s1 - s0) / mm) + 1, nt = static_cast<int>((t1 - t0) / mm) + 1;
    if (static_cast<double>(ns) * nt > c.max_cells) {continue;}
    cv::Mat u(nt, ns, CV_32F), v(nt, ns, CV_32F), Zg(nt, ns, CV_64F), tol(nt, ns, CV_64F);
    for (int iy = 0; iy < nt; ++iy) {
      const double tt = t0 + mm * iy;
      for (int ix = 0; ix < ns; ++ix) {
        const double ss = s0 + mm * ix;
        cv::Vec3d X;
        for (int i = 0; i < 3; ++i) {X[i] = (c0[i] + ss * e1[i]) + tt * e2[i];}
        const double Z = X[2];
        u.at<float>(iy, ix) = static_cast<float>(K.fx * X[0] / Z + K.cx);
        v.at<float>(iy, ix) = static_cast<float>(K.fy * X[1] / Z + K.cy);
        Zg.at<double>(iy, ix) = Z;
        // 평면 위인지: 허용치 = 고정 depth_tol + 이 평면에서 잰 잡음 × tol_k (거리 제곱에 비례해 늘림)
        const double zr = Z / pl.zmed;
        if (c.tol_inc) {
          // sig 는 평면 법선 방향 잡음, 비교는 시선 방향 깊이 차 → 1/cos(입사각) 배(비스듬한 면일수록 큼)
          const double ci = std::fabs((X[0] * n[0] + X[1] * n[1]) + X[2] * n[2]) /
            std::max(std::sqrt((X[0] * X[0] + X[1] * X[1]) + X[2] * X[2]), 1e-6);
          tol.at<double>(iy, ix) = c.depth_tol + c.tol_k * pl.sig * (zr * zr) / std::max(ci, 0.2);
        } else {
          tol.at<double>(iy, ix) = c.depth_tol + c.tol_k * pl.sig * (zr * zr);
        }
      }
    }
    cv::Mat Rg, Rs, Dd;
    cv::remap(g, Rg, u, v, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    cv::remap(sat, Rs, u, v, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    cv::remap(zf, Dd, u, v, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::Mat valid(nt, ns, CV_32F);
    for (int iy = 0; iy < nt; ++iy) {
      for (int ix = 0; ix < ns; ++ix) {
        const float uu = u.at<float>(iy, ix), vv = v.at<float>(iy, ix);
        const double Z = Zg.at<double>(iy, ix);
        valid.at<float>(iy, ix) = (uu >= 0 && uu < W - 1 && vv >= 0 && vv < H - 1 && Z > c.depth_min &&
          std::fabs(Dd.at<float>(iy, ix) - Z) < tol.at<double>(iy, ix)) ? 1.0f : 0.0f;
      }
    }
    // 카메라 화소 하나가 평면에서 덮는 넓이(비스듬하면 한쪽으로 길다)만큼 템플릿을 흐리게 한다
    auto psf_kernel = [&](const cv::Vec3d & Xp) -> std::optional<cv::Mat> {
        const cv::Vec2d p0 = proj(Xp);
        const cv::Vec2d j1 = (proj(Xp + 1e-3 * e1) - p0) / 1e-3, j2 = (proj(Xp + 1e-3 * e2) - p0) / 1e-3;
        const cv::Matx22d J(j1[0], j2[0], j1[1], j2[1]);
        const cv::Matx22d Ji = lapack_inv2(J);
        cv::Matx22d Sig = Ji * Ji.t();   // 펴진 영상 칸 단위 (Py: Ji @ Ji.T * psf_px ** 2 / mm ** 2)
        for (int i = 0; i < 4; ++i) {Sig.val[i] = Sig.val[i] * (c.psf_px * c.psf_px) / (mm * mm);}
        const double r_f = 3 * std::sqrt(lapack_eigmax2(Sig));
        // 평면이 시선과 거의 나란하면 화소 하나가 평면에서 아주 길게 늘어나 흐림 커널이 수만 칸이 됨(메모리 수십 GB).
        // 그런 평면에서는 라벨(20 mm)이 몇 화소도 안 되어 찾을 수 없으므로 건너뜀
        if (!std::isfinite(r_f) || r_f > c.max_blur) {return std::nullopt;}
        const int rk = static_cast<int>(std::ceil(r_f));
        const cv::Matx22d Si = lapack_inv2(Sig);
        cv::Mat k_(2 * rk + 1, 2 * rk + 1, CV_32F);
        for (int ky = -rk; ky <= rk; ++ky) {
          for (int kx = -rk; kx <= rk; ++kx) {
            const double q = Si(0, 0) * kx * kx + 2 * Si(0, 1) * kx * ky + Si(1, 1) * ky * ky;
            k_.at<float>(ky + rk, kx + rk) = static_cast<float>(std::exp(-0.5 * q));
          }
        }
        div_by_sum(k_);
        return k_;
      };
    const auto ker = psf_kernel(c0 + npc::np_median(sv) * e1 + npc::np_median(tv) * e2);
    if (!ker) {continue;}
    // 라벨 창 전체가 평면 경계에서 조금 안쪽(win_erode 칸)에 있어야 함: 경계 칸은 옆 면 밝기가 섞임
    // (새 Mat 에 받는다: valid 와 버퍼를 같이 쓰는 헤더에 erode 하면 valid 까지 깎임)
    cv::Mat valid_w;
    if (c.win_erode) {
      cv::erode(valid, valid_w, cv::Mat::ones(2 * c.win_erode + 1, 2 * c.win_erode + 1, CV_8U));
    } else {
      valid_w = valid;
    }
    cv::Mat best(nt, ns, CV_32F, cv::Scalar(-1.0f)), bidx(nt, ns, CV_32S, cv::Scalar(0));
    std::vector<cv::Mat> tpb(tpls_.size());   // 이 평면에서 흐린 템플릿(후보 검사에 다시 씀)
    const cv::Mat RgRg = Rg.mul(Rg);
    // 회전 하나의 점수: 정규 상관, 창 안 평면 위 칸 비율 · 밝기 표준편차가 모자라면 −1
    auto score_one = [&](size_t ti, cv::Mat & sc) {
        const auto & tp = tpls_[ti];
        cv::Mat vf, mu, m2;
        cv::matchTemplate(Rg, tpb[ti], sc, cv::TM_CCOEFF_NORMED, tp.mk);
        const float area = static_cast<float>(cv::countNonZero(tp.mk));
        cv::matchTemplate(valid_w, tp.mk, vf, cv::TM_CCORR);
        cv::matchTemplate(Rg, tp.mk, mu, cv::TM_CCORR);
        cv::matchTemplate(RgRg, tp.mk, m2, cv::TM_CCORR);
        for (int y = 0; y < sc.rows; ++y) {
          for (int x = 0; x < sc.cols; ++x) {
            float & s = sc.at<float>(y, x);
            if (!std::isfinite(s)) {s = -1.0f;}
            const float mu_ = mu.at<float>(y, x) / area, m2_ = m2.at<float>(y, x) / area;
            const float sd = std::sqrt(std::max(m2_ - mu_ * mu_, 0.0f));
            if (vf.at<float>(y, x) / area < static_cast<float>(c.valid_frac) || sd < static_cast<float>(c.min_std)) {s = -1.0f;}
          }
        }
      };
    const int D = tpls_.front().tp.rows;   // 회전 템플릿은 모두 D × D
    if (nt < D || ns < D) {
      cv::filter2D(tpls_.front().tp, tpb.front(), -1, *ker, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    } else {
      // 회전끼리는 서로 독립이라 몇 개씩 함께 계산하고, 최댓값 고르기는 Python 과 같은 회전 순서로 한다(같은 결과)
      const double cells = static_cast<double>(nt) * ns;
      const int batch = std::clamp(static_cast<int>(256e6 / (16.0 * cells)), 1, std::max(1, cv::getNumThreads()));
      const int h0 = D / 2;
      for (size_t b0 = 0; b0 < tpls_.size(); b0 += static_cast<size_t>(batch)) {
        const size_t b1 = std::min(tpls_.size(), b0 + static_cast<size_t>(batch));
        std::vector<cv::Mat> scs(b1 - b0);
        cv::parallel_for_(cv::Range(static_cast<int>(b0), static_cast<int>(b1)), [&](const cv::Range & rr) {
            for (int ti = rr.start; ti < rr.end; ++ti) {
              cv::filter2D(tpls_[static_cast<size_t>(ti)].tp, tpb[static_cast<size_t>(ti)], -1, *ker, cv::Point(-1, -1), 0,
                cv::BORDER_REPLICATE);
              score_one(static_cast<size_t>(ti), scs[static_cast<size_t>(ti) - b0]);
            }
          });
        for (size_t ti = b0; ti < b1; ++ti) {
          const cv::Mat & sc = scs[ti - b0];
          for (int y = 0; y < sc.rows; ++y) {
            for (int x = 0; x < sc.cols; ++x) {
              const float s = sc.at<float>(y, x);
              float & bb = best.at<float>(y + h0, x + h0);
              if (s > bb) {
                bb = s;
                bidx.at<int>(y + h0, x + h0) = static_cast<int>(ti);
              }
            }
          }
        }
      }
    }
    // 봉우리: 둘레 nms 안 최댓값
    const int r = std::max(1, static_cast<int>(c.nms / mm));
    cv::Mat dil, valid_er;
    cv::dilate(best, dil, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1)));
    cv::erode(valid, valid_er, cv::Mat::ones(5, 5, CV_8U));   // 평면 가장자리에서 섞인 칸 빼기
    const float thr = static_cast<float>(c.thr - c.rescore_m);   // 봉우리 문턱 (thr 바로 아래는 다시 잼)
    for (int ty = 0; ty < nt; ++ty) {
      for (int sx = 0; sx < ns; ++sx) {
        const float bv = best.at<float>(ty, sx);
        if (!(bv >= thr && bv >= dil.at<float>(ty, sx))) {continue;}
        const int ti = bidx.at<int>(ty, sx);
        const Template & tpl = tpls_[static_cast<size_t>(ti)];
        // 둘레 띠: 라벨은 무늬 없는 면에 붙인 스티커 → 바깥 띠(평면 위 칸만)는 고르고, 라벨 안은 무늬가 있다
        const float thf = npc::np_radiansf(static_cast<float>(tpl.theta));
        const float a[2] = {npc::np_cosf(thf), -npc::np_sinf(thf)}, b[2] = {npc::np_sinf(thf), npc::np_cosf(thf)};
        auto rect = [&](double grow) {
            const double gw = lw / 2 + grow / c.mm, gh = lh / 2 + grow / c.mm;
            const int ij[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
            std::vector<cv::Point> pts;
            for (const auto & q : ij) {
              const double base[2] = {static_cast<double>(sx), static_cast<double>(ty)};
              int pt[2];
              for (int d = 0; d < 2; ++d) {
                // Py: np.array([sx, ty]) + i * gw * a + j * gh * b (스칼라 × float32 배열은 float32)
                const float ia = static_cast<float>(q[0] * gw) * a[d], jb = static_cast<float>(q[1] * gh) * b[d];
                pt[d] = static_cast<int>(std::nearbyint((base[d] + ia) + jb));
              }
              pts.emplace_back(pt[0], pt[1]);
            }
            return pts;
          };
        cv::Mat m_lab = cv::Mat::zeros(nt, ns, CV_8U), m_in = m_lab.clone(), m_out = m_lab.clone();
        const auto r0 = rect(0);
        cv::fillPoly(m_lab, std::vector<std::vector<cv::Point>>{r0}, 1);
        cv::fillPoly(m_in, std::vector<std::vector<cv::Point>>{rect(c.ring_in)}, 1);
        cv::fillPoly(m_out, std::vector<std::vector<cv::Point>>{rect(c.ring_out)}, 1);
        cv::Mat ring = m_out & (m_in == 0);
        cv::Mat rv = ring & (valid_er > 0);
        cv::Mat lab_v = m_lab & (valid > 0);
        const int n_rv = cv::countNonZero(rv), n_ring = cv::countNonZero(ring), n_labv = cv::countNonZero(lab_v);
        const double ring_valid = static_cast<double>(n_rv) / std::max(n_ring, 1);
        const std::vector<float> ring_vals = masked(Rg, rv);
        double ring_std = n_rv > 10 ? npc::np_std_f32(ring_vals) : 1e9;
        const double ring_mean = n_rv > 10 ? npc::np_mean_f32(ring_vals) : 0.0;
        // 라벨 흰 테두리(템플릿 밝은 곳) 평균과 둘레 비교: 스티커 흰색은 둘레 면보다 어둡지 않다
        const int y0 = ty - D / 2, x0 = sx - D / 2;
        if (y0 < 0 || x0 < 0 || y0 + D > nt || x0 + D > ns) {continue;}
        const cv::Rect wr(x0, y0, D, D);
        const cv::Mat win = Rg(wr);
        const double white_mean = npc::np_mean_f32(masked(win, tpl.bright));
        const double band_mean = npc::np_mean_f32(masked(win, tpl.dark));
        const cv::Mat mk8 = tpl.mk > 0.5f;
        const double sat_mean = npc::np_mean_f32(masked(Rs(wr), mk8));
        double score = bv;
        cv::Mat tpb_c = tpb[static_cast<size_t>(ti)], ker_c = *ker;
        if (score < c.thr) {
          // 문턱 바로 아래 후보: 라벨 앞을 물체가 조금 가리면(평면 밖 칸) 그 칸이 상관을 끌어내림 → 평면 위 칸만으로 다시 잰다
          const cv::Mat mv = mk8 & (valid(wr) > 0);
          score = ncc(masked(win, mv), masked(tpb_c, mv));
          if (score < c.thr) {continue;}
        }
        if (c.local_psf) {
          // 흐림은 평면 한 점(가운데)에서 구했으므로, 후보 자리에서 다시 구해 점수 · 특징을 다시 잰다
          const auto kc = psf_kernel(c0 + (s0 + mm * sx) * e1 + (t0 + mm * ty) * e2);
          if (!kc) {continue;}
          ker_c = *kc;
          cv::Mat t;   // 새 Mat (tpb_c 는 다른 후보도 쓰는 tpb[ti] 와 버퍼를 같이 씀)
          cv::filter2D(tpl.tp, t, -1, ker_c, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
          tpb_c = t;
          score = ncc(masked(win, mk8), masked(tpb_c, mk8));
          if (score < c.thr) {continue;}
        }
        const ShapeFeats feat = shape_feats(win, tpb_c, tpl.tp, tpl.mk, ker_c, tpl.bright, tpl.theta, lw, lh);
        // 라벨 창이 평면에서 떠 있는 높이(시선 방향 깊이 차의 중앙값, m): 스티커는 거의 0
        std::vector<double> lifts;
        const cv::Mat & lv = n_labv > 10 ? lab_v : m_lab;
        for (int y = 0; y < nt; ++y) {
          for (int x = 0; x < ns; ++x) {
            if (lv.at<uchar>(y, x)) {lifts.push_back(Zg.at<double>(y, x) - Dd.at<float>(y, x));}
          }
        }
        const double lift = npc::np_median(lifts);
        if (lift > c.lift_max) {continue;}
        // 바코드 라벨은 흑백 인쇄 → 채도가 낮다(빨강 · 초록 퍽, 파란 기둥과 다름)
        if (sat_mean > c.sat_max) {continue;}
        // 평균이 낮아도 창 일부에 색 물체(퍽)가 들어오면 안 됨: 라벨 안쪽(가장자리 2 칸 뺌) 채도 상위 10 %
        cv::Mat m_core;
        cv::erode(m_lab, m_core, cv::Mat::ones(5, 5, CV_8U));
        const double sat_hi = cv::countNonZero(m_core) > 10 ? npc::np_percentile(masked(Rs, m_core), 90.0) : 1e9;
        if (sat_hi > c.sat_hi_max) {continue;}
        // 막대 띠는 검은 막대와 흰 빈칸이 섞여 회색: 태그처럼 새까만 곳은 아니다
        if (band_mean < c.bw_min * white_mean) {continue;}
        // 흰 스티커는 같은 조명을 받는 둘레 면보다 어둡지 않다: 물체 옆 그림자는 '흰 테두리' 자리까지 둘레보다 어둡다
        if (white_mean < c.white_ring_min * ring_mean) {continue;}
        // 막대 띠는 흰 테두리보다 뚜렷이 어둡다(검은 막대): 흐린 그림자 · 얼룩은 명암 차가 작다
        if (band_mean > c.bw_max * white_mean) {continue;}
        // 무늬 명암비: 흐린 템플릿에 맞춘 명암 폭(fit_a × 템플릿 흰색 255)을 흰 테두리 밝기로 나눈 값.
        // 바코드는 막대가 검정보다 어두울 수 없어 1 근처가 상한. 비스듬히 본 장소 태그는 넓은 검은 칸이
        // 흐린 막대 모형보다 진해 1 을 넘는다(판정 4차 헛검출 1.15 · 1.23, 설계 자료 진짜 최대 1.004)
        const double con = feat.fit_a * 255.0 / std::max(white_mean, 1.0);
        if (con > c.con_max) {continue;}
        // 둘레가 고른가: 네 변(모서리 제외)으로 나눠 가장 얼룩진 한 변은 빼고 본다
        //   (라벨 옆에 기둥 · 퍽이 붙어 있어도 한 변만 얼룩짐. 태그 가장자리 등은 여러 변이 얼룩짐)
        {
          const double hw = lw / 2, hh = lh / 2;
          std::vector<double> la, lb;
          std::vector<float> vals;
          std::array<std::vector<char>, 4> sides;
          for (int y = 0; y < nt; ++y) {
            for (int x = 0; x < ns; ++x) {
              if (!rv.at<uchar>(y, x)) {continue;}
              const double l1 = static_cast<double>(x - sx) * a[0] + static_cast<double>(y - ty) * a[1];
              const double l2 = static_cast<double>(x - sx) * b[0] + static_cast<double>(y - ty) * b[1];
              la.push_back(l1);
              lb.push_back(l2);
              vals.push_back(Rg.at<float>(y, x));
              sides[0].push_back(l2 < -hh && std::fabs(l1) <= hw);
              sides[1].push_back(l2 > hh && std::fabs(l1) <= hw);
              sides[2].push_back(l1 < -hw && std::fabs(l2) <= hh);
              sides[3].push_back(l1 > hw && std::fabs(l2) <= hh);
            }
          }
          // 변마다 둘레 칸 전체(평면 밖 포함) 수: 거의 다 평면 밖인 변(남은 몇 칸은 평면 경계)은 빼고 본다
          std::array<int, 4> full{0, 0, 0, 0};
          if (c.side_min_frac > 0) {
            for (int y = 0; y < nt; ++y) {
              for (int x = 0; x < ns; ++x) {
                if (!ring.at<uchar>(y, x)) {continue;}
                const double l1 = static_cast<double>(x - sx) * a[0] + static_cast<double>(y - ty) * a[1];
                const double l2 = static_cast<double>(x - sx) * b[0] + static_cast<double>(y - ty) * b[1];
                full[0] += l2 < -hh && std::fabs(l1) <= hw;
                full[1] += l2 > hh && std::fabs(l1) <= hw;
                full[2] += l1 < -hw && std::fabs(l2) <= hh;
                full[3] += l1 > hw && std::fabs(l2) <= hh;
              }
            }
          }
          std::vector<const std::vector<char> *> ok_sides;
          for (size_t i = 0; i < 4; ++i) {
            const auto n_s = std::count(sides[i].begin(), sides[i].end(), 1);
            if (n_s >= 6 && (c.side_min_frac <= 0 || static_cast<double>(n_s) >= c.side_min_frac * full[i])) {
              ok_sides.push_back(&sides[i]);
            }
          }
          auto pick = [&](const std::vector<char> & m) {
              std::vector<float> o;
              for (size_t i = 0; i < vals.size(); ++i) {if (m[i]) {o.push_back(vals[i]);}}
              return o;
            };
          if (c.ring_mad) {   // 중앙값 기반 퍼짐: 이웃 물체가 둘레 일부(절반 미만)를 차지해도 흔들리지 않음
            const float md = npc::np_median(vals);
            std::vector<float> dev(vals.size());
            for (size_t i = 0; i < vals.size(); ++i) {dev[i] = std::fabs(vals[i] - md);}
            ring_std = 1.4826 * static_cast<double>(npc::np_median(dev));
          } else if (c.ring_drop_worst && ok_sides.size() >= 3) {
            size_t worst = 0;
            float wstd = -1.0f;
            for (size_t i = 0; i < ok_sides.size(); ++i) {
              const float sd = npc::np_std_f32(pick(*ok_sides[i]));
              if (sd > wstd) {wstd = sd; worst = i;}
            }
            std::vector<char> keep(vals.size(), 0);
            for (size_t i = 0; i < ok_sides.size(); ++i) {
              if (i == worst) {continue;}
              for (size_t j = 0; j < vals.size(); ++j) {keep[j] |= (*ok_sides[i])[j];}
            }
            ring_std = npc::np_std_f32(pick(keep));
            if (c.ring_lin) {   // 무늬 없는 면도 조명 때문에 밝기가 완만히 기움 → 기울기(1 차 평면)를 빼고 남은 퍼짐
              std::vector<int> ix;
              for (size_t j = 0; j < vals.size(); ++j) {if (keep[j]) {ix.push_back(static_cast<int>(j));}}
              const size_t m = ix.size();
              std::vector<double> A(3 * m), y(m);
              for (size_t q = 0; q < m; ++q) {
                A[q] = 1.0;
                A[m + q] = la[static_cast<size_t>(ix[q])];
                A[2 * m + q] = lb[static_cast<size_t>(ix[q])];
                y[q] = vals[static_cast<size_t>(ix[q])];
              }
              const auto cf = lapack_lstsq(static_cast<int>(m), 3, A, y);
              std::vector<double> res(m);   // Py: vals − A @ cf (dgemv, 차례 합)
              for (size_t q = 0; q < m; ++q) {res[q] = y[q] - ((A[q] * cf[0] + A[m + q] * cf[1]) + A[2 * m + q] * cf[2]);}
              ring_std = npc::np_std_f64(res);
            }
          } else if (c.side_min_frac > 0 && !ok_sides.empty()) {   // 남은 변이 한두 개면 그 변들만으로
            std::vector<char> keep(vals.size(), 0);
            for (const auto * m : ok_sides) {
              for (size_t j = 0; j < vals.size(); ++j) {keep[j] |= (*m)[j];}
            }
            ring_std = npc::np_std_f32(pick(keep));
          }
        }
        const double ring_c = ring_std / std::max(white_mean - band_mean, 1.0);
        if (ring_c > c.ring_max) {continue;}
        // 둘레는 무늬 없는 면이라 조명 밝기에 견줘도 고르다(태그 안쪽에 걸린 후보는 둘레가 태그 무늬라 얼룩짐)
        if (ring_std > c.ring_rel_max * std::max(white_mean, 1.0)) {continue;}
        // 흰 테두리 자리에 템플릿 예상보다 아주 어두운 칸이 있으면 흰 스티커가 아님(태그의 검은 칸)
        if (feat.resw_lo < c.resw_min) {continue;}
        // 가장자리가 카메라 흐림보다 더 흐리면 인쇄물이 아님(그림자 · 음영): 두 배 흐린 템플릿이 더 잘 맞음
        if (feat.sharp < c.sharp_min) {continue;}
        // 흰 테두리 한 변 전체가 예상보다 어두우면 흰 스티커가 아님(물체 옆 접촉 그림자 · 음영 기울기)
        if (feat.side_lo < c.side_min) {continue;}
        // 라벨 창이 영상 안에 다 들어와야 함(영상 끝에 잘린 물체를 라벨로 보지 않게)
        bool inside = true;
        for (const auto & q : r0) {
          const cv::Vec2d p = proj(c0 + (s0 + mm * q.x) * e1 + (t0 + mm * q.y) * e2);
          inside = inside && c.img_margin <= p[0] && p[0] < W - c.img_margin && c.img_margin <= p[1] && p[1] < H - c.img_margin;
        }
        if (!inside) {continue;}
        if (ring_valid < c.ring_min_valid) {continue;}
        BarcodeDetection d;
        d.surface = pl.kind;
        d.center = c0 + (s0 + mm * sx) * e1 + (t0 + mm * ty) * e2;
        d.normal = n;
        d.score = score;
        d.theta = tpl.theta;
        const cv::Vec2d px = proj(d.center);
        d.px = {px[0], px[1]};
        d.sat = sat_mean;
        d.ring_c = ring_c;
        d.ring_std = ring_std;
        d.ring_mean = ring_mean;
        d.ring_valid = ring_valid;
        d.white_mean = white_mean;
        d.band_mean = band_mean;
        d.sat_hi = sat_hi;
        d.lift = lift;
        d.pl_w = pl_w;
        d.resw_lo = feat.resw_lo;
        d.side_lo = feat.side_lo;
        d.sharp = feat.sharp;
        d.fit_a = feat.fit_a;
        d.con = con;
        out.push_back(d);
      }
    }
  }
  std::stable_sort(out.begin(), out.end(), [](const BarcodeDetection & x, const BarcodeDetection & y) {return x.score > y.score;});
  std::vector<BarcodeDetection> kept;
  for (const auto & d : out) {
    bool far = true;
    for (const auto & o : kept) {far = far && cv::norm(d.center - o.center) >= c.nms;}
    if (far) {kept.push_back(d);}
  }
  return kept;
}

}  // namespace perception
