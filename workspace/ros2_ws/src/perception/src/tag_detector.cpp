#include "perception/tag_detector.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/version.hpp>

#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define ROBOT_ARUCO_NEW 1
#else
#include <opencv2/aruco.hpp>
#define ROBOT_ARUCO_NEW 0
#endif

namespace perception
{

TagDetector::TagDetector(const std::string & family, double tag_size_m)
: size_(tag_size_m)
{
  ok_ = true;
  if (family == "tag36h11") {dict_id_ = cv::aruco::DICT_APRILTAG_36h11;}
  else if (family == "tag25h9") {dict_id_ = cv::aruco::DICT_APRILTAG_25h9;}
  else if (family == "tag16h5") {dict_id_ = cv::aruco::DICT_APRILTAG_16h5;}
  else if (family == "DICT_4X4_50") {dict_id_ = cv::aruco::DICT_4X4_50; aruco_ = true;}
  else if (family == "DICT_4X4_100") {dict_id_ = cv::aruco::DICT_4X4_100; aruco_ = true;}
  else if (family == "DICT_5X5_50") {dict_id_ = cv::aruco::DICT_5X5_50; aruco_ = true;}
  else if (family == "DICT_6X6_50") {dict_id_ = cv::aruco::DICT_6X6_50; aruco_ = true;}
  else {ok_ = false;}
}

namespace
{
// 태그 z 축(면에서 바깥)이 카메라 쪽을 향하는지. t 는 카메라 → 태그 중심
bool facing(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  return R(0, 2) * tvec[0] + R(1, 2) * tvec[1] + R(2, 2) * tvec[2] < 0.0;
}

// IPPE_SQUARE 두 해 중 카메라를 향하는 것 가운데 재투영 오차가 작은 것.
// 태그를 거의 정면에서 보면 IPPE 가 뒷면을 보는 해(법선이 카메라 반대)만 내는 일이 있어(2026-10-23 모의 시험,
// 0° 에서 퍽 중심이 면 반대쪽으로 계산됨) 그때는 반복법으로 다시 푼다
bool solve_facing(const std::vector<cv::Point3f> & obj, const std::vector<cv::Point2f> & img, const cv::Mat & K,
  const cv::Mat & D, cv::Vec3d & rvec, cv::Vec3d & tvec)
{
  std::vector<cv::Mat> rs, ts;
  cv::Mat err;
  const int n = cv::solvePnPGeneric(obj, img, K, D, rs, ts, false, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(), err);
  err.convertTo(err, CV_64F);   // 입력 점이 float 이면 CV_32F 로 나온다
  int best = -1;
  for (int k = 0; k < n; ++k) {
    const cv::Vec3d r(rs[k]), t(ts[k]);
    if (!facing(r, t)) {continue;}
    if (best < 0 || err.at<double>(k) < err.at<double>(best)) {best = k;}
  }
  if (best >= 0) {
    rvec = cv::Vec3d(rs[best]);
    tvec = cv::Vec3d(ts[best]);
    return true;
  }
  if (!cv::solvePnP(obj, img, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {return false;}
  return facing(rvec, tvec);
}
}  // namespace

std::vector<TagDetection> TagDetector::detect(const cv::Mat & bgr, const cv::Mat & K, const cv::Mat & D) const
{
  std::vector<TagDetection> out;
  if (!ok_ || bgr.empty()) {return out;}
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  // AprilTag 은 두 방식으로 찾아 번호별로 합친다: CORNER_REFINE_APRILTAG(AprilTag 사각형 찾기)와
  // CORNER_REFINE_SUBPIX(윤곽 찾기)가 놓치는 태그가 서로 달라서(2026-10-23 합성 영상 144조건: 113 · 102 → 합쳐서 128).
  // 같은 번호면 AprilTag 방식 결과를 쓴다. Aruco 마커는 SUBPIX 한 번
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<std::vector<cv::Point2f>> cand;   // 원래 크기 영상에서 사각형은 찾았지만 해독하지 못한 후보
  bool keep_cand = true;
  auto run = [&](int method) {
      std::vector<int> ii;
      std::vector<std::vector<cv::Point2f>> cc, rejected;
#if ROBOT_ARUCO_NEW
      cv::aruco::DetectorParameters p;
      p.cornerRefinementMethod = method;
      cv::aruco::ArucoDetector det(cv::aruco::getPredefinedDictionary(dict_id_), p);
      det.detectMarkers(gray, cc, ii, rejected);
#else
      auto p = cv::aruco::DetectorParameters::create();
      p->cornerRefinementMethod = method;
      cv::aruco::detectMarkers(gray, cv::aruco::getPredefinedDictionary(dict_id_), cc, ii, p, rejected);
#endif
      for (size_t k = 0; k < ii.size(); ++k) {
        if (std::find(ids.begin(), ids.end(), ii[k]) != ids.end()) {continue;}
        ids.push_back(ii[k]);
        corners.push_back(cc[k]);
      }
      if (keep_cand) {cand.insert(cand.end(), rejected.begin(), rejected.end());}
    };
  if (aruco_) {
    run(cv::aruco::CORNER_REFINE_SUBPIX);
  } else {
    run(cv::aruco::CORNER_REFINE_APRILTAG);
    run(cv::aruco::CORNER_REFINE_SUBPIX);
  }
  // 하나도 못 찾으면 2배로 키운 영상에서 한 번 더 찾고 꼭짓점을 원래 크기로 줄인다.
  // D435 1 m 의 48 mm 장소 태그(약 29 px)를 밝은 조명에서 놓친 7장이 2배에서 모두 찾아짐(R5 렌더 2026-10-25)
  keep_cand = false;
  const bool none_at_1x = ids.empty();
  if (ids.empty()) {
    const cv::Mat orig = gray;
    cv::resize(orig, gray, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
    if (aruco_) {
      run(cv::aruco::CORNER_REFINE_SUBPIX);
    } else {
      run(cv::aruco::CORNER_REFINE_APRILTAG);
      run(cv::aruco::CORNER_REFINE_SUBPIX);
    }
    for (auto & cs : corners) {
      for (auto & c : cs) {c = (c + cv::Point2f(0.5f, 0.5f)) * 0.5f - cv::Point2f(0.5f, 0.5f);}   // 화소 가운데 기준
    }
    gray = orig;
  }
  // 다른 태그는 찾아 위의 2배 재검출이 돌지 않았으면, 해독하지 못한 사각형 후보 둘레만 2배로 키워 다시 해독한다.
  // 팔 카메라 0.10 m 의 16 mm 퍽 태그(약 45 px)를 밝은 조명에서 사각형은 찾고 해독만 못 한 1장이 2배에서 읽힘(R4 렌더 2026-10-25).
  // 한 변 8~200 px 후보 중 큰 것부터 최대 10개, 둘레 여백 = 긴 변의 반
  if (!none_at_1x && !cand.empty()) {
    std::vector<std::pair<double, cv::Rect>> rois;
    const cv::Rect img(0, 0, gray.cols, gray.rows);
    for (const auto & q : cand) {
      if (q.size() != 4) {continue;}
      double lo = 1e9, hi = 0.0;
      for (int k = 0; k < 4; ++k) {
        const double e = cv::norm(q[k] - q[(k + 1) % 4]);
        lo = std::min(lo, e);
        hi = std::max(hi, e);
      }
      if (lo < 8.0 || hi > 200.0) {continue;}
      const int m = static_cast<int>(std::ceil(hi / 2.0));
      const cv::Rect r = (cv::boundingRect(q) + cv::Size(2 * m, 2 * m) - cv::Point(m, m)) & img;
      const bool same = std::any_of(rois.begin(), rois.end(), [&](const auto & o) {return (o.second & r).area() > 0.8 * r.area();});
      if (!same && r.area() > 0) {rois.emplace_back(std::fabs(cv::contourArea(q)), r);}
    }
    std::sort(rois.begin(), rois.end(), [](const auto & a, const auto & b) {return a.first > b.first;});
    if (rois.size() > 10) {rois.resize(10);}
    const cv::Mat orig = gray;
    for (const auto & [area, r] : rois) {
      cv::Mat up;   // 새 버퍼에 키움: 키운 크기가 원래 영상과 같아도 orig 를 덮어쓰지 않게
      cv::resize(orig(r), up, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
      gray = up;
      const size_t n0 = corners.size();
      if (aruco_) {
        run(cv::aruco::CORNER_REFINE_SUBPIX);
      } else {
        run(cv::aruco::CORNER_REFINE_APRILTAG);
        run(cv::aruco::CORNER_REFINE_SUBPIX);
      }
      for (size_t k = n0; k < corners.size(); ++k) {
        for (auto & c : corners[k]) {c = (c + cv::Point2f(0.5f, 0.5f)) * 0.5f - cv::Point2f(0.5f, 0.5f) + cv::Point2f(r.tl());}
      }
    }
    gray = orig;
  }
  const double h = size_ / 2.0;
  // aruco 모서리 순서: 왼쪽 위 · 오른쪽 위 · 오른쪽 아래 · 왼쪽 아래 (태그 평면 z = 0)
  const std::vector<cv::Point3f> obj = {{float(-h), float(h), 0}, {float(h), float(h), 0},
    {float(h), float(-h), 0}, {float(-h), float(-h), 0}};
  for (size_t i = 0; i < ids.size(); ++i) {
    TagDetection t;
    t.id = ids[i];
    t.area = std::fabs(cv::contourArea(corners[i]));
    t.corners = corners[i];
    if (!K.empty()) {
      t.pose_valid = solve_facing(obj, corners[i], K, D.empty() ? cv::Mat::zeros(1, 5, CV_64F) : D, t.rvec, t.tvec);
    }
    out.push_back(t);
  }
  return out;
}

}  // namespace perception
