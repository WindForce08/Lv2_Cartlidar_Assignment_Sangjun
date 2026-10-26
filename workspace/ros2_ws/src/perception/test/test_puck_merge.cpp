// 카메라 사이 퍽 병합 시험 (puck_merge.hpp). 퍽 폭 30 mm, merge_dist 0.04 m
#include <gtest/gtest.h>

#include "perception/puck_merge.hpp"

using namespace perception;

namespace
{
constexpr double D = 0.04;
constexpr int UNK = 0, RED = 1, GREEN = 2;
constexpr int FRONT = 0, REAR = 1, ARM = 2;

PuckObs tag(double x, int color, int id) {return {x, 0.0, 0.10, color, id, ARM, 0.9};}
PuckObs cam(double x, int color, int camera, double conf = 0.8) {return {x, 0.0, 0.10, color, -1, camera, conf};}
}  // namespace

// 태그 A 와 그 D435 관측 A'(5 mm), 35 mm 옆 다른 퍽 B → A 와 B 가 남는다 (예전: 40 mm 안이라 B 도 뺐음)
TEST(PuckMerge, NeighborDifferentPuckStays)
{
  const std::vector<PuckObs> obs{tag(0.300, RED, 3), cam(0.305, RED, FRONT), cam(0.335, RED, FRONT)};
  EXPECT_EQ(merge_pucks(obs, D), (std::vector<size_t>{0, 2}));
}

TEST(PuckMerge, SamePuckKeepsTag)
{
  const std::vector<PuckObs> obs{tag(0.300, RED, 3), cam(0.310, RED, FRONT)};
  EXPECT_EQ(merge_pucks(obs, D), (std::vector<size_t>{0}));
}

TEST(PuckMerge, DifferentColorNotMerged)
{
  const std::vector<PuckObs> obs{tag(0.300, RED, 3), cam(0.320, GREEN, FRONT)};
  EXPECT_EQ(merge_pucks(obs, D), (std::vector<size_t>{0, 1}));
}

// 앞 · 뒤가 같은 퍽 → confidence 높은 쪽 하나. 같은 카메라 안의 두 퍽은 묶지 않는다
TEST(PuckMerge, FrontRearDuplicateKeepsHigherConfidence)
{
  const std::vector<PuckObs> obs{cam(0.500, GREEN, FRONT, 0.7), cam(0.515, GREEN, REAR, 0.9), cam(0.535, GREEN, FRONT, 0.8)};
  EXPECT_EQ(merge_pucks(obs, D), (std::vector<size_t>{1, 2}));
}

// 태그 · 앞 · 뒤가 한 퍽을 보면 태그 퍽 하나
TEST(PuckMerge, TagFrontRearSamePuck)
{
  const std::vector<PuckObs> obs{tag(0.300, RED, 3), cam(0.315, RED, FRONT), cam(0.290, RED, REAR)};
  EXPECT_EQ(merge_pucks(obs, D), (std::vector<size_t>{0}));
}

// 색 모름은 어느 색과도 묶는다(사용자 결정 2026-10-26). 묶을 거리 밖이면 남긴다
TEST(PuckMerge, UnknownColorMatchesAny)
{
  const std::vector<PuckObs> obs{tag(0.300, UNK, 3), cam(0.310, GREEN, FRONT), cam(0.400, RED, REAR)};
  EXPECT_EQ(merge_pucks(obs, D), (std::vector<size_t>{0, 2}));
}
