#ifndef TERRAIN_ANALYSIS__TRAVERSABLE_TERRAIN_HPP_
#define TERRAIN_ANALYSIS__TRAVERSABLE_TERRAIN_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace terrain_analysis
{

// Intensity for points released as traversable transition (surface / riser).
// Negative so local_planner (threshold ~0.3) treats them as free, while FAR can
// mark jump-allowed terrain cells.
constexpr float kTransitionFreeIntensity = -1.0f;

struct TraversableTerrainParams
{
  float support_band_eps = 0.06f;
  int min_support_points = 3;
  float wall_bin_size = 0.10f;
  float wall_fill_thre = 0.55f;
  float vehicle_height = 1.5f;
  int seq_length = 4;
  float flat_eps = 0.08f;
  float min_seq_rise = 0.06f;
  float min_step_rise = 0.10f;  // |Δh| below this does not count as a step
  float max_step_rise = 0.30f;  // single-step rise above this is not traversable
  int min_significant_steps = 2;  // need at least this many rises (rejects single bumps)
  int min_width_cells = 2;
  float width_height_eps = 0.08f;
  float surface_eps = 0.06f;
};

inline int CellIndex(int width, int x, int y)
{
  return width * x + y;
}

inline bool InGrid(int width, int x, int y)
{
  return x >= 0 && x < width && y >= 0 && y < width;
}

// Points near elev count as support; heavy vertical fill rejects walls/pillars.
inline void MarkSupportCells(
  const float * elevations,
  const std::vector<float> * point_zs,
  int width,
  int min_block_points,
  const TraversableTerrainParams & params,
  bool * support_cells)
{
  const int cell_count = width * width;
  std::fill(support_cells, support_cells + cell_count, false);
  if (width <= 0) {
    return;
  }

  const int bin_count = std::max(
    1, static_cast<int>(std::ceil(params.vehicle_height / params.wall_bin_size)));
  constexpr int kMaxBins = 64;
  const int use_bins = std::min(bin_count, kMaxBins);

  for (int i = 0; i < cell_count; ++i) {
    const auto & zs = point_zs[i];
    if (static_cast<int>(zs.size()) < min_block_points) {
      continue;
    }

    const float elev = elevations[i];
    int support_count = 0;
    uint8_t bins[kMaxBins] = {0};
    for (float z : zs) {
      if (std::fabs(z - elev) <= params.support_band_eps) {
        ++support_count;
      }
      const float above = z - elev;
      if (above > 0.0f && above < params.vehicle_height) {
        int bin = static_cast<int>(above / params.wall_bin_size);
        if (bin < 0) {
          bin = 0;
        }
        if (bin >= use_bins) {
          bin = use_bins - 1;
        }
        bins[bin] = 1;
      }
    }

    int filled = 0;
    for (int b = 0; b < use_bins; ++b) {
      filled += bins[b];
    }
    const float fill = static_cast<float>(filled) / static_cast<float>(use_bins);
    if (support_count >= params.min_support_points && fill < params.wall_fill_thre) {
      support_cells[i] = true;
    }
  }
}

inline bool SequenceMonotonic(
  const float * elevations, const int * indices, int length, float flat_eps,
  float min_seq_rise, float min_step_rise, float max_step_rise,
  int min_significant_steps)
{
  if (length < 2) {
    return false;
  }
  const float total = elevations[indices[length - 1]] - elevations[indices[0]];
  if (std::fabs(total) < min_seq_rise) {
    return false;
  }
  const int sign = total > 0.0f ? 1 : -1;
  int significant_steps = 0;
  for (int i = 1; i < length; ++i) {
    const float d = elevations[indices[i]] - elevations[indices[i - 1]];
    const float abs_d = std::fabs(d);
    if (abs_d <= flat_eps) {
      continue;
    }
    if ((d > 0.0f ? 1 : -1) != sign) {
      return false;
    }
    // Too tall for the robot to treat as a traversable step.
    if (abs_d > max_step_rise) {
      return false;
    }
    // Too small to count as a real step (noise / gentle ripple).
    if (abs_d < min_step_rise) {
      continue;
    }
    ++significant_steps;
  }
  return significant_steps >= min_significant_steps;
}

inline bool HasLateralWidth(
  const float * elevations, const bool * support_cells, int width, int x, int y,
  int dir_x, int dir_y, const TraversableTerrainParams & params)
{
  // Perpendicular to (dir_x, dir_y); for diagonals use a cardinal normal.
  int nx = -dir_y;
  int ny = dir_x;
  if (nx == 0 && ny == 0) {
    return false;
  }
  // Normalize 2-length diagonal normals already unit in grid steps.
  if (std::abs(nx) > 1 || std::abs(ny) > 1) {
    nx = (nx == 0) ? 0 : (nx > 0 ? 1 : -1);
    ny = (ny == 0) ? 0 : (ny > 0 ? 1 : -1);
  }

  const float elev0 = elevations[CellIndex(width, x, y)];
  int width_count = 1;
  for (int s = 1; s <= params.min_width_cells; ++s) {
    for (int sign : {-1, 1}) {
      const int lx = x + sign * s * nx;
      const int ly = y + sign * s * ny;
      if (!InGrid(width, lx, ly)) {
        continue;
      }
      const int li = CellIndex(width, lx, ly);
      if (!support_cells[li]) {
        continue;
      }
      if (std::fabs(elevations[li] - elev0) <= params.width_height_eps) {
        ++width_count;
      }
    }
  }
  return width_count >= params.min_width_cells;
}

// Mark transition cells: support surface + directed height run + lateral width.
inline void MarkTransitionCells(
  const float * elevations, const bool * support_cells, int width,
  const TraversableTerrainParams & params, bool * transition_cells,
  int8_t * primary_dx = nullptr, int8_t * primary_dy = nullptr)
{
  const int cell_count = width * width;
  std::fill(transition_cells, transition_cells + cell_count, false);
  if (primary_dx) {
    std::fill(primary_dx, primary_dx + cell_count, static_cast<int8_t>(0));
  }
  if (primary_dy) {
    std::fill(primary_dy, primary_dy + cell_count, static_cast<int8_t>(0));
  }
  if (width <= 0 || params.seq_length < 2) {
    return;
  }

  static const int kDirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  const int L = params.seq_length;
  std::vector<int> indices(static_cast<size_t>(L), 0);

  for (int i = 0; i < cell_count; ++i) {
    if (!support_cells[i]) {
      continue;
    }
    const int x = i / width;
    const int y = i % width;

    for (const auto & dir : kDirs) {
      const int dx = dir[0];
      const int dy = dir[1];
      bool ok = true;
      for (int s = 0; s < L; ++s) {
        const int sx = x + s * dx;
        const int sy = y + s * dy;
        if (!InGrid(width, sx, sy)) {
          ok = false;
          break;
        }
        const int si = CellIndex(width, sx, sy);
        if (!support_cells[si]) {
          ok = false;
          break;
        }
        indices[static_cast<size_t>(s)] = si;
      }
      if (!ok) {
        continue;
      }
      if (!SequenceMonotonic(
          elevations, indices.data(), L, params.flat_eps, params.min_seq_rise,
          params.min_step_rise, params.max_step_rise,
          params.min_significant_steps))
      {
        continue;
      }
      bool width_ok = true;
      for (int s = 0; s < L; ++s) {
        const int si = indices[static_cast<size_t>(s)];
        const int sx = si / width;
        const int sy = si % width;
        if (!HasLateralWidth(
            elevations, support_cells, width, sx, sy, dx, dy, params))
        {
          width_ok = false;
          break;
        }
      }
      if (!width_ok) {
        continue;
      }

      for (int s = 0; s < L; ++s) {
        const int si = indices[static_cast<size_t>(s)];
        transition_cells[si] = true;
        if (primary_dx) {
          primary_dx[si] = static_cast<int8_t>(dx);
        }
        if (primary_dy) {
          primary_dy[si] = static_cast<int8_t>(dy);
        }
      }
    }
  }
}

inline float NeighborSupportElevRange(
  const float * elevations, const bool * support_cells, int width, int cell,
  float & out_min, float & out_max)
{
  const int x = cell / width;
  const int y = cell % width;
  out_min = elevations[cell];
  out_max = elevations[cell];
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const int nx = x + dx;
      const int ny = y + dy;
      if (!InGrid(width, nx, ny)) {
        continue;
      }
      const int ni = CellIndex(width, nx, ny);
      if (!support_cells[ni]) {
        continue;
      }
      out_min = std::min(out_min, elevations[ni]);
      out_max = std::max(out_max, elevations[ni]);
    }
  }
  return out_max - out_min;
}

// Point-level release on transition cells only.
inline float PointIntensity(
  float point_z, float cell_elev, float dis_z, bool is_transition,
  const float * elevations, const bool * support_cells, int width, int cell,
  const TraversableTerrainParams & params)
{
  if (!is_transition) {
    return dis_z;
  }

  if (std::fabs(point_z - cell_elev) <= params.surface_eps) {
    return kTransitionFreeIntensity;
  }

  float adj_min = cell_elev;
  float adj_max = cell_elev;
  NeighborSupportElevRange(
    elevations, support_cells, width, cell, adj_min, adj_max);

  if (point_z >= adj_min - params.surface_eps &&
    point_z <= adj_max + params.surface_eps)
  {
    return kTransitionFreeIntensity;
  }

  return dis_z;
}

// Convenience: full pipeline for one grid snapshot.
inline void DetectTraversableTransitions(
  const float * elevations,
  const std::vector<float> * point_zs,
  int width,
  int min_block_points,
  const TraversableTerrainParams & params,
  bool * support_cells,
  bool * transition_cells)
{
  MarkSupportCells(
    elevations, point_zs, width, min_block_points, params, support_cells);
  MarkTransitionCells(
    elevations, support_cells, width, params, transition_cells);
}

}  // namespace terrain_analysis

#endif  // TERRAIN_ANALYSIS__TRAVERSABLE_TERRAIN_HPP_
