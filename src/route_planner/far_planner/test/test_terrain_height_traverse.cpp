#include <gtest/gtest.h>

#include <vector>

#include "far_planner/terrain_height_traverse.hpp"

TEST(TerrainHeightTraverse, MatchingHeightsPassWithoutJump)
{
  const std::vector<float> ref{1.0f, 1.02f};
  float out = 0.0f;
  EXPECT_TRUE(far_planner::IsTerrainHeightNeighborTraversable(
    1.0f, ref, 0.1f, false, false, false, &out));
  EXPECT_NEAR(1.01f, out, 1e-3);
}

TEST(TerrainHeightTraverse, LargeGapBlockedWithoutJumpFlag)
{
  const std::vector<float> ref{2.0f};
  float out = 0.0f;
  EXPECT_FALSE(far_planner::IsTerrainHeightNeighborTraversable(
    1.0f, ref, 0.1f, false, false, false, &out));
}

TEST(TerrainHeightTraverse, GlobalAllowStillRequiresJumpCell)
{
  const std::vector<float> ref{2.0f};
  float out = 0.0f;
  EXPECT_FALSE(far_planner::IsTerrainHeightNeighborTraversable(
    1.0f, ref, 0.1f, true, false, false, &out));
}

TEST(TerrainHeightTraverse, JumpCellAllowsLargeGap)
{
  const std::vector<float> ref{2.0f, 2.2f};
  float out = 0.0f;
  EXPECT_TRUE(far_planner::IsTerrainHeightNeighborTraversable(
    1.0f, ref, 0.1f, true, true, false, &out));
  EXPECT_NEAR(2.1f, out, 1e-3);

  EXPECT_TRUE(far_planner::IsTerrainHeightNeighborTraversable(
    1.0f, ref, 0.1f, true, false, true, &out));
}
