#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "terrain_analysis/traversable_terrain.hpp"

namespace
{

struct GridCase
{
  int width;
  std::vector<float> elevations;
  std::vector<std::vector<float>> point_zs;
};

GridCase MakeEmpty(int width)
{
  GridCase g;
  g.width = width;
  g.elevations.assign(static_cast<size_t>(width * width), 0.0f);
  g.point_zs.resize(static_cast<size_t>(width * width));
  return g;
}

void AddSupportPoints(GridCase & g, int x, int y, float elev, int count = 8)
{
  const int i = terrain_analysis::CellIndex(g.width, x, y);
  g.elevations[static_cast<size_t>(i)] = elev;
  g.point_zs[static_cast<size_t>(i)].clear();
  for (int k = 0; k < count; ++k) {
    g.point_zs[static_cast<size_t>(i)].push_back(elev);
  }
}

void AddWallColumn(GridCase & g, int x, int y, float ground, float height)
{
  const int i = terrain_analysis::CellIndex(g.width, x, y);
  g.elevations[static_cast<size_t>(i)] = ground;
  g.point_zs[static_cast<size_t>(i)].clear();
  g.point_zs[static_cast<size_t>(i)].push_back(ground);
  g.point_zs[static_cast<size_t>(i)].push_back(ground);
  for (float z = ground + 0.1f; z <= ground + height; z += 0.1f) {
    g.point_zs[static_cast<size_t>(i)].push_back(z);
  }
}

std::vector<bool> RunDetect(const GridCase & g, terrain_analysis::TraversableTerrainParams params =
                                                  terrain_analysis::TraversableTerrainParams())
{
  std::vector<char> support(g.elevations.size(), 0);
  std::vector<char> transition(g.elevations.size(), 0);
  terrain_analysis::DetectTraversableTransitions(
    g.elevations.data(), g.point_zs.data(), g.width, 3, params,
    reinterpret_cast<bool *>(support.data()),
    reinterpret_cast<bool *>(transition.data()));
  std::vector<bool> out(g.elevations.size(), false);
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = transition[i] != 0;
  }
  return out;
}

TEST(TraversableTerrain, FlatGroundHasNoTransition)
{
  auto g = MakeEmpty(7);
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 7; ++y) {
      AddSupportPoints(g, x, y, 0.0f);
    }
  }
  const auto result = RunDetect(g);
  EXPECT_EQ(0, std::count(result.begin(), result.end(), true));
}

TEST(TraversableTerrain, MultiStepStairsAreDetected)
{
  auto g = MakeEmpty(9);
  for (int x = 0; x < 9; ++x) {
    for (int y = 0; y < 9; ++y) {
      const float elev = 0.15f * static_cast<float>(x);
      AddSupportPoints(g, x, y, elev);
    }
  }
  const auto result = RunDetect(g);
  EXPECT_GT(std::count(result.begin(), result.end(), true), 20);
}

TEST(TraversableTerrain, DiagonalStairsAreDetected)
{
  auto g = MakeEmpty(9);
  for (int x = 0; x < 9; ++x) {
    for (int y = 0; y < 9; ++y) {
      AddSupportPoints(g, x, y, 0.0f);
    }
  }
  // Wide diagonal band of rising steps.
  for (int x = 1; x < 8; ++x) {
    for (int y = x - 1; y <= x + 1; ++y) {
      if (y < 0 || y >= 9) {
        continue;
      }
      AddSupportPoints(g, x, y, 0.15f * static_cast<float>(x));
    }
  }
  const auto result = RunDetect(g);
  EXPECT_GT(std::count(result.begin(), result.end(), true), 5);
}

TEST(TraversableTerrain, SmallRisesDoNotCountAsSteps)
{
  auto g = MakeEmpty(9);
  for (int x = 0; x < 9; ++x) {
    for (int y = 0; y < 9; ++y) {
      // 0.07m per cell: above flat_eps=0.08? No — below flat if flat=0.08.
      // Use 0.09: above flat_eps but below min_step_rise=0.10 → not counted.
      const float elev = 0.09f * static_cast<float>(x);
      AddSupportPoints(g, x, y, elev);
    }
  }
  terrain_analysis::TraversableTerrainParams params;
  params.flat_eps = 0.08f;
  params.min_step_rise = 0.10f;
  params.min_significant_steps = 2;
  params.min_seq_rise = 0.10f;
  const auto result = RunDetect(g, params);
  EXPECT_EQ(0, std::count(result.begin(), result.end(), true));
}

TEST(TraversableTerrain, TallStepsAreRejected)
{
  auto g = MakeEmpty(9);
  for (int x = 0; x < 9; ++x) {
    for (int y = 0; y < 9; ++y) {
      // 0.35m per cell exceeds default max_step_rise=0.30
      const float elev = 0.35f * static_cast<float>(x);
      AddSupportPoints(g, x, y, elev);
    }
  }
  const auto result = RunDetect(g);
  EXPECT_EQ(0, std::count(result.begin(), result.end(), true));
}

TEST(TraversableTerrain, TallStepsPassWhenMaxRaised)
{
  auto g = MakeEmpty(9);
  for (int x = 0; x < 9; ++x) {
    for (int y = 0; y < 9; ++y) {
      const float elev = 0.35f * static_cast<float>(x);
      AddSupportPoints(g, x, y, elev);
    }
  }
  terrain_analysis::TraversableTerrainParams params;
  params.max_step_rise = 0.40f;
  const auto result = RunDetect(g, params);
  EXPECT_GT(std::count(result.begin(), result.end(), true), 20);
}

TEST(TraversableTerrain, SingleStepRejectedByMinSteps)
{
  auto g = MakeEmpty(7);
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 7; ++y) {
      // One wide curb: only one significant rise in any length-4 window.
      AddSupportPoints(g, x, y, x >= 3 ? 0.15f : 0.0f);
    }
  }
  terrain_analysis::TraversableTerrainParams params;
  params.min_significant_steps = 2;
  const auto result = RunDetect(g, params);
  EXPECT_EQ(0, std::count(result.begin(), result.end(), true));
}

TEST(TraversableTerrain, RaisedBoxIsRejected)
{
  auto g = MakeEmpty(9);
  for (int x = 0; x < 9; ++x) {
    for (int y = 0; y < 9; ++y) {
      AddSupportPoints(g, x, y, 0.0f);
    }
  }
  for (int x = 3; x <= 5; ++x) {
    for (int y = 3; y <= 5; ++y) {
      AddSupportPoints(g, x, y, 0.25f);
    }
  }
  const auto result = RunDetect(g);
  EXPECT_EQ(0, std::count(result.begin(), result.end(), true));
}

TEST(TraversableTerrain, NarrowPillarIsRejected)
{
  auto g = MakeEmpty(7);
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 7; ++y) {
      AddSupportPoints(g, x, y, 0.0f);
    }
  }
  for (int x = 0; x < 7; ++x) {
    AddSupportPoints(g, x, 3, 0.15f * static_cast<float>(x));
  }
  const auto result = RunDetect(g);
  EXPECT_EQ(0, std::count(result.begin(), result.end(), true));
}

TEST(TraversableTerrain, WallColumnIsNotSupport)
{
  auto g = MakeEmpty(5);
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      AddSupportPoints(g, x, y, 0.0f);
    }
  }
  AddWallColumn(g, 2, 2, 0.0f, 1.2f);
  std::vector<char> support(g.elevations.size(), 0);
  std::vector<char> transition(g.elevations.size(), 0);
  terrain_analysis::DetectTraversableTransitions(
    g.elevations.data(), g.point_zs.data(), g.width, 3,
    terrain_analysis::TraversableTerrainParams(),
    reinterpret_cast<bool *>(support.data()),
    reinterpret_cast<bool *>(transition.data()));
  EXPECT_FALSE(support[terrain_analysis::CellIndex(5, 2, 2)]);
}

TEST(TraversableTerrain, ShortCurbMayMissWithDefaultSeq)
{
  auto g = MakeEmpty(7);
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 7; ++y) {
      AddSupportPoints(g, x, y, x >= 3 ? 0.20f : 0.0f);
    }
  }
  const auto result = RunDetect(g);
  // Single step has only one significant rise along a length-4 window.
  EXPECT_EQ(0, std::count(result.begin(), result.end(), true));
}

TEST(TraversableTerrain, PointReleaseKeepsHighObstacles)
{
  auto g = MakeEmpty(9);
  for (int x = 0; x < 9; ++x) {
    for (int y = 0; y < 9; ++y) {
      AddSupportPoints(g, x, y, 0.15f * static_cast<float>(x));
    }
  }
  std::vector<char> support(g.elevations.size(), 0);
  std::vector<char> transition(g.elevations.size(), 0);
  terrain_analysis::TraversableTerrainParams params;
  terrain_analysis::DetectTraversableTransitions(
    g.elevations.data(), g.point_zs.data(), g.width, 3, params,
    reinterpret_cast<bool *>(support.data()),
    reinterpret_cast<bool *>(transition.data()));

  const int cell = terrain_analysis::CellIndex(9, 4, 4);
  ASSERT_TRUE(transition[static_cast<size_t>(cell)]);
  const float elev = g.elevations[static_cast<size_t>(cell)];
  const float surface = terrain_analysis::PointIntensity(
    elev, elev, 0.0f, true, g.elevations.data(),
    reinterpret_cast<const bool *>(support.data()), 9, cell, params);
  EXPECT_FLOAT_EQ(terrain_analysis::kTransitionFreeIntensity, surface);

  const float riser = terrain_analysis::PointIntensity(
    elev - 0.08f, elev, 0.08f, true, g.elevations.data(),
    reinterpret_cast<const bool *>(support.data()), 9, cell, params);
  EXPECT_FLOAT_EQ(terrain_analysis::kTransitionFreeIntensity, riser);

  const float obstacle = terrain_analysis::PointIntensity(
    elev + 0.8f, elev, 0.8f, true, g.elevations.data(),
    reinterpret_cast<const bool *>(support.data()), 9, cell, params);
  EXPECT_FLOAT_EQ(0.8f, obstacle);
}

}  // namespace
