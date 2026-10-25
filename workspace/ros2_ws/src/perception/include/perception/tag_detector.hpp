// AprilTag (tag36h11) 검출 · 자세 추정 — OpenCV aruco 의 AprilTag 사전 사용 (별도 apriltag 라이브러리 불필요)
//   OpenCV ≥ 4.7: objdetect 의 cv::aruco::ArucoDetector / 4.6: contrib aruco::detectMarkers
//   자세: solvePnP(IPPE_SQUARE) — 광학 좌표(x 오른쪽 · y 아래 · z 앞)
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct TagDetection
{
  int id = -1;
  double area = 0;            // 화소 넓이 (가장 크게 보이는 태그 고르기)
  std::vector<cv::Point2f> corners;   // 왼쪽 위 · 오른쪽 위 · 오른쪽 아래 · 왼쪽 아래 (화소)
  bool pose_valid = false;
  cv::Vec3d rvec, tvec;       // 광학 좌표, m
  double err = 0;             // rvec · tvec 의 재투영 오차 [px] (IPPE)
  bool alt_valid = false;     // 카메라를 향하는 IPPE 의 다른 해 (작은 태그는 두 해의 오차가 거의 같음 → tag_upright.hpp)
  cv::Vec3d rvec_alt, tvec_alt;
  double err_alt = 0;
};

class TagDetector
{
public:
  // family: "tag36h11" (기본) · "tag25h9" · "tag16h5"
  //         Aruco(과제 마커): "DICT_4X4_50" · "DICT_4X4_100" · "DICT_5X5_50" · "DICT_6X6_50"
  TagDetector(const std::string & family, double tag_size_m);
  bool ok() const {return ok_;}
  // K · D 가 비어 있으면 자세 추정 없이 id 만
  std::vector<TagDetection> detect(const cv::Mat & bgr, const cv::Mat & K, const cv::Mat & D) const;

private:
  int dict_id_ = 0;
  double size_ = 0.1;
  bool ok_ = false;
  bool aruco_ = false;
};

}  // namespace perception
