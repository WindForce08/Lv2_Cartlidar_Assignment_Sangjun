// READ_TAG 결과로 낼 태그 고르기 (ROS 의존 없음 → 단위 시험)
//
//   퍽 태그(tag.* 크기, tag.ids)와 장소 태그(marker.* 크기, marker.ids)는 같은 tag36h11 계열이지만 크기가 달라
//   각자의 검출기로 자세를 구한다. 두 목록을 합쳐 화면에서 가장 크게 보이는 것 하나를 tag_id · tag_pose 로 낸다
//   (같은 거리면 48 mm 장소 태그가 16 mm 퍽 태그보다 넓이 9배라 함께 보이면 장소 태그가 뽑힌다)
#pragma once

#include <algorithm>
#include <vector>

#include "perception/tag_detector.hpp"

namespace perception
{

// ids 에 없는 번호를 뺀다. ids 가 비면 모두 남김
inline void keep_tag_ids(std::vector<TagDetection> & tags, const std::vector<int> & ids)
{
  if (ids.empty()) {return;}
  tags.erase(std::remove_if(tags.begin(), tags.end(), [&ids](const TagDetection & t) {
      return std::find(ids.begin(), ids.end(), t.id) == ids.end();}), tags.end());
}

// 두 목록 중 넓이가 가장 큰 태그 (같으면 pucks 쪽 · 앞선 것). 없으면 nullptr
inline const TagDetection * largest_tag(const std::vector<TagDetection> & pucks, const std::vector<TagDetection> & places)
{
  const TagDetection * best = nullptr;
  for (const auto * l : {&pucks, &places}) {
    for (const auto & t : *l) {if (!best || t.area > best->area) {best = &t;}}
  }
  return best;
}

}  // namespace perception
