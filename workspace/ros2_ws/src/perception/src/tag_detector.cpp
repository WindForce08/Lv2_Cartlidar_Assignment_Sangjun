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
      t.pose_valid = cv::solvePnP(obj, corners[i], K, D.empty() ? cv::Mat::zeros(1, 5, CV_64F) : D, t.rvec, t.tvec,
          false, cv::SOLVEPNP_IPPE_SQUARE);
    }
    out.push_back(t);
  }
  return out;
}

}  // namespace perception
