#include "perception/tag_detector.hpp"

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
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
#if ROBOT_ARUCO_NEW
  cv::aruco::DetectorParameters p;
  p.cornerRefinementMethod = aruco_ ? cv::aruco::CORNER_REFINE_SUBPIX : cv::aruco::CORNER_REFINE_APRILTAG;
  cv::aruco::ArucoDetector det(cv::aruco::getPredefinedDictionary(dict_id_), p);
  det.detectMarkers(gray, corners, ids, rejected);
#else
  auto p = cv::aruco::DetectorParameters::create();
  p->cornerRefinementMethod = aruco_ ? cv::aruco::CORNER_REFINE_SUBPIX : cv::aruco::CORNER_REFINE_APRILTAG;
  cv::aruco::detectMarkers(gray, cv::aruco::getPredefinedDictionary(dict_id_), corners, ids, p, rejected);
#endif
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
