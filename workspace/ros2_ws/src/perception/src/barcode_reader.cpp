// 바코드 문자열 읽기 (barcode_reader.hpp). zbar C API
#include "perception/barcode_reader.hpp"

#include <cmath>
#include <set>

#include <opencv2/imgproc.hpp>
#include <zbar.h>

namespace perception
{
using namespace zbar;   // C++ 로 읽으면 zbar.h 의 C 함수가 zbar 이름공간 안에 있다

BarcodeRead decide_barcode(const std::vector<std::string> & symbols)
{
  BarcodeRead r;
  r.symbols = symbols;
  const std::set<std::string> u(symbols.begin(), symbols.end());
  if (u.size() == 1) {
    r.ok = true;
    r.text = *u.begin();
  }
  return r;
}

BarcodeReader::BarcodeReader(double scale, double gamma, int min_quality, int x_density, int y_density)
: scale_(scale > 0 ? scale : 1.0), min_quality_(min_quality)
{
  if (gamma > 0 && gamma != 1.0) {   // Py: (255 * (np.arange(256) / 255.0) ** GAMMA).round()
    lut_.create(1, 256, CV_8U);
    for (int i = 0; i < 256; ++i) {lut_.at<uchar>(i) = static_cast<uchar>(std::nearbyint(255 * std::pow(i / 255.0, gamma)));}
  }
  auto * sc = zbar_image_scanner_create();
  zbar_image_scanner_set_config(sc, ZBAR_NONE, ZBAR_CFG_ENABLE, 0);      // 모두 끔
  zbar_image_scanner_set_config(sc, ZBAR_CODE128, ZBAR_CFG_ENABLE, 1);   // CODE128 켬
  zbar_image_scanner_set_config(sc, ZBAR_NONE, ZBAR_CFG_X_DENSITY, x_density);
  zbar_image_scanner_set_config(sc, ZBAR_NONE, ZBAR_CFG_Y_DENSITY, y_density);
  scanner_ = sc;
}

BarcodeReader::~BarcodeReader()
{
  if (scanner_) {zbar_image_scanner_destroy(static_cast<zbar_image_scanner_t *>(scanner_));}
}

BarcodeRead BarcodeReader::read(const cv::Mat & img)
{
  std::vector<std::string> syms;
  if (img.empty()) {return decide_barcode(syms);}
  cv::Mat g;
  if (img.channels() == 3) {
    cv::cvtColor(img, g, cv::COLOR_BGR2GRAY);
  } else if (img.type() == CV_8U) {
    g = img.isContinuous() ? img : img.clone();
  } else {
    return decide_barcode(syms);
  }
  if (!lut_.empty()) {
    cv::Mat t;
    cv::LUT(g, lut_, t);
    g = t;
  }
  if (scale_ != 1.0) {
    cv::Mat big;
    cv::resize(g, big, cv::Size(), scale_, scale_, cv::INTER_CUBIC);
    g = big;
  }
  zbar_image_t * zi = zbar_image_create();
  zbar_image_set_format(zi, zbar_fourcc('Y', '8', '0', '0'));
  zbar_image_set_size(zi, static_cast<unsigned>(g.cols), static_cast<unsigned>(g.rows));
  zbar_image_set_data(zi, g.data, g.total(), nullptr);
  zbar_scan_image(static_cast<zbar_image_scanner_t *>(scanner_), zi);
  int dropped = 0;
  for (const zbar_symbol_t * s = zbar_image_first_symbol(zi); s; s = zbar_symbol_next(s)) {
    if (zbar_symbol_get_quality(s) >= min_quality_) {
      syms.emplace_back(zbar_symbol_get_data(s), zbar_symbol_get_data_length(s));
    } else {
      ++dropped;
    }
  }
  zbar_image_destroy(zi);
  BarcodeRead r = decide_barcode(syms);
  r.dropped = dropped;
  return r;
}

}  // namespace perception
