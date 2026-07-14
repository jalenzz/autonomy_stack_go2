#ifndef FAR_PLANNER__TERRAIN_HEIGHT_TRAVERSE_HPP_
#define FAR_PLANNER__TERRAIN_HEIGHT_TRAVERSE_HPP_

#include <cmath>
#include <vector>

namespace far_planner
{

// Decide whether ref terrain cell is traversable from cur height.
// When allow_jumps is set, height gaps are accepted only if either cell is a
// transition/jump cell (marked from terrain_analysis free intensity < 0).
inline bool IsTerrainHeightNeighborTraversable(
  float cur_h,
  const std::vector<float> & ref_heights,
  float height_thred,
  bool allow_jumps,
  bool cur_is_jump_cell,
  bool ref_is_jump_cell,
  float * out_ref_h)
{
  if (ref_heights.empty()) {
    return false;
  }

  if (allow_jumps && (cur_is_jump_cell || ref_is_jump_cell)) {
    float sum = 0.0f;
    for (float e : ref_heights) {
      sum += e;
    }
    if (out_ref_h) {
      *out_ref_h = sum / static_cast<float>(ref_heights.size());
    }
    return true;
  }

  float ref_h = 0.0f;
  int counter = 0;
  for (float e : ref_heights) {
    if (std::fabs(e - cur_h) > height_thred) {
      continue;
    }
    ref_h += e;
    ++counter;
  }
  if (counter > 0) {
    if (out_ref_h) {
      *out_ref_h = ref_h / static_cast<float>(counter);
    }
    return true;
  }
  return false;
}

}  // namespace far_planner

#endif  // FAR_PLANNER__TERRAIN_HEIGHT_TRAVERSE_HPP_
