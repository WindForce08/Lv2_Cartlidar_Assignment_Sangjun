// barcode_compare: 바코드 찾기(BarcodeLocator)를 렌더 장면 목록에 돌려 JSON 줄로 낸다 (ROS 없이)
//   scripts/barcode_compare.py 가 기준 Python(locate_v2.py) 출력과 견줄 때 쓴다.
//   찾기: barcode_compare <장면 목록> <라벨 그림> [키=값 ...] [--planes]   (키 = BarcodeLocatorConfig 이름, 예: thr=0.7)
//     장면 목록 한 줄: <이름> <컬러 png> <깊이 npy(float32, m)> fx fy cx cy <T_base_cam 16 개(행 순서)>
//     출력(표준 출력) 한 줄: {"name": …, "ms": 걸린 시간, "dets": [{"kind", "center", "score", "theta", "px", …}], "planes": [...]}
//   읽기: barcode_compare --read <영상 목록> [배율 [감마 [최소 quality]]]   (한 줄: <이름> <영상 png>; 기본 1 · 2 · 4)
//     출력 한 줄: {"name": …, "ms": …, "symbols": [...], "ok": …, "text": …}
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <type_traits>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "perception/barcode_locator.hpp"
#include "perception/barcode_reader.hpp"

using perception::BarcodeDetection;
using perception::BarcodeLocator;
using perception::BarcodeLocatorConfig;

namespace
{
// float32 · float64 C 순서 .npy (2 차원) 만
cv::Mat load_npy(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  char magic[6];
  f.read(magic, 6);
  if (!f || std::memcmp(magic, "\x93NUMPY", 6) != 0) {return {};}
  unsigned char ver[2];
  f.read(reinterpret_cast<char *>(ver), 2);
  uint32_t hlen = 0;
  if (ver[0] == 1) {
    unsigned char b[2];
    f.read(reinterpret_cast<char *>(b), 2);
    hlen = b[0] | (b[1] << 8);
  } else {
    unsigned char b[4];
    f.read(reinterpret_cast<char *>(b), 4);
    hlen = b[0] | (b[1] << 8) | (b[2] << 16) | (static_cast<uint32_t>(b[3]) << 24);
  }
  std::string hdr(hlen, ' ');
  f.read(&hdr[0], hlen);
  const bool f8 = hdr.find("'<f8'") != std::string::npos;
  if ((!f8 && hdr.find("'<f4'") == std::string::npos) || hdr.find("'fortran_order': True") != std::string::npos) {return {};}
  const auto p = hdr.find("'shape': (");
  int H = 0, W = 0;
  if (p == std::string::npos || std::sscanf(hdr.c_str() + p + 10, "%d, %d", &H, &W) != 2) {return {};}
  cv::Mat m(H, W, f8 ? CV_64F : CV_32F);
  f.read(reinterpret_cast<char *>(m.data), static_cast<std::streamsize>(m.total() * m.elemSize()));
  if (!f) {return {};}
  if (f8) {m.convertTo(m, CV_32F);}
  return m;
}

bool set_param(BarcodeLocatorConfig & c, const std::string & k, const std::string & v)
{
  bool found = false;
  perception::visit_barcode_params(c, [&](const char * name, auto & x) {
      if (k != name) {return;}
      found = true;
      using T = std::decay_t<decltype(x)>;
      if constexpr (std::is_same_v<T, bool>) {
        x = v == "1" || v == "true" || v == "True";
      } else if constexpr (std::is_same_v<T, int>) {
        x = std::stoi(v);
      } else {
        x = std::stod(v);
      }
    });
  return found;
}

std::string quote(const std::string & s)
{
  std::string o = "\"";
  for (char ch : s) {
    if (ch == '"' || ch == '\\') {o += '\\'; o += ch;} else if (static_cast<unsigned char>(ch) < 0x20) {
      char b[8];
      std::snprintf(b, sizeof(b), "\\u%04x", ch);
      o += b;
    } else {o += ch;}
  }
  return o + "\"";
}

double ms_since(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

int run_read(const char * list_path, double scale, double gamma, int qmin)
{
  perception::BarcodeReader reader(scale, gamma, qmin);
  std::ifstream list(list_path);
  std::string line;
  while (std::getline(list, line)) {
    if (line.empty()) {continue;}
    std::istringstream ss(line);
    std::string name, path;
    ss >> name >> path;
    const cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
    if (img.empty()) {std::cerr << "읽기 실패: " << name << "\n"; return 1;}
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = reader.read(img);
    std::ostringstream o;
    o << "{\"name\": " << quote(name) << ", \"ms\": " << ms_since(t0) << ", \"symbols\": [";
    for (size_t i = 0; i < r.symbols.size(); ++i) {o << (i ? ", " : "") << quote(r.symbols[i]);}
    o << "], \"dropped\": " << r.dropped << ", \"ok\": " << (r.ok ? "true" : "false") << ", \"text\": " << quote(r.text) << "}";
    std::cout << o.str() << std::endl;
  }
  return 0;
}

std::string num(double x)
{
  if (!std::isfinite(x)) {return "null";}
  char b[40];
  std::snprintf(b, sizeof(b), "%.17g", x);
  return b;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc >= 3 && argc <= 6 && std::string(argv[1]) == "--read") {
    return run_read(argv[2], argc > 3 ? std::stod(argv[3]) : 1.0, argc > 4 ? std::stod(argv[4]) : 2.0, argc > 5 ? std::stoi(argv[5]) : 4);
  }
  if (argc < 3) {
    std::cerr << "사용: barcode_compare <장면 목록> <라벨 그림> [키=값 ...]\n";
    return 2;
  }
  BarcodeLocatorConfig cfg;
  bool planes_out = false;
  for (int a = 3; a < argc; ++a) {
    const std::string s = argv[a];
    if (s == "--planes") {planes_out = true; continue;}
    const auto eq = s.find('=');
    if (eq == std::string::npos || !set_param(cfg, s.substr(0, eq), s.substr(eq + 1))) {
      std::cerr << "모르는 인자: " << s << "\n";
      return 2;
    }
  }
  const BarcodeLocator loc(cfg, std::string(argv[2]));
  if (!loc.ok()) {
    std::cerr << "라벨 그림을 읽지 못함: " << argv[2] << "\n";
    return 1;
  }
  std::ifstream list(argv[1]);
  std::string line;
  while (std::getline(list, line)) {
    if (line.empty()) {continue;}
    std::istringstream ss(line);
    std::string name, color, depth;
    perception::BarcodeK K;
    cv::Matx44d T;
    ss >> name >> color >> depth >> K.fx >> K.fy >> K.cx >> K.cy;
    for (int i = 0; i < 16; ++i) {ss >> T.val[i];}
    if (!ss) {std::cerr << "목록 줄 오류: " << line << "\n"; return 1;}
    const cv::Mat bgr = cv::imread(color, cv::IMREAD_COLOR);
    const cv::Mat dep = load_npy(depth);
    if (bgr.empty() || dep.empty()) {std::cerr << "읽기 실패: " << name << "\n"; return 1;}
    const auto t0 = std::chrono::steady_clock::now();
    const auto dets = loc.locate(bgr, dep, K, T);
    const double ms = ms_since(t0);
    std::ostringstream o;
    o << "{\"name\": " << quote(name) << ", \"ms\": " << ms << ", \"dets\": [";
    for (size_t i = 0; i < dets.size(); ++i) {
      const auto & d = dets[i];
      o << (i ? ", " : "") << "{\"kind\": \"" << (d.surface == BarcodeDetection::WALL ? "wall" : "top") << "\", \"center\": ["
        << num(d.center[0]) << ", " << num(d.center[1]) << ", " << num(d.center[2]) << "], \"score\": " << num(d.score)
        << ", \"theta\": " << num(d.theta) << ", \"px\": [" << num(d.px.x) << ", " << num(d.px.y) << "]"
        << ", \"sat\": " << num(d.sat) << ", \"ring_c\": " << num(d.ring_c) << ", \"ring_std\": " << num(d.ring_std)
        << ", \"ring_mean\": " << num(d.ring_mean) << ", \"white_mean\": " << num(d.white_mean)
        << ", \"band_mean\": " << num(d.band_mean) << ", \"sharp\": " << num(d.sharp) << ", \"side_lo\": " << num(d.side_lo)
        << ", \"resw_lo\": " << num(d.resw_lo) << ", \"lift\": " << num(d.lift) << ", \"pl_w\": " << num(d.pl_w) << ", \"con\": " << num(d.con) << "}";
    }
    o << "]";
    if (planes_out) {
      const auto pls = loc.surfaces(dep, K, T);
      o << ", \"planes\": [";
      for (size_t i = 0; i < pls.size(); ++i) {
        const auto & p = pls[i];
        o << (i ? ", " : "") << "{\"kind\": \"" << (p.kind == BarcodeDetection::WALL ? "wall" : "top") << "\", \"n_px\": "
          << p.ys.size() << ", \"c\": [" << num(p.c[0]) << ", " << num(p.c[1]) << ", " << num(p.c[2]) << "], \"n\": ["
          << num(p.n[0]) << ", " << num(p.n[1]) << ", " << num(p.n[2]) << "], \"rms\": " << num(p.rms) << ", \"sig\": "
          << num(p.sig) << ", \"zmed\": " << num(p.zmed) << ", \"ztop\": " << num(p.ztop) << "}";
      }
      o << "]";
    }
    o << "}";
    std::cout << o.str() << std::endl;
  }
  return 0;
}
