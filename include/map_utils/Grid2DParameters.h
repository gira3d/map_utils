#pragma once

namespace map_utils
{
  namespace grid2d
  {
    typedef struct Parameters
    {
      unsigned int width;
      unsigned int height;

      float resolution;
      Point origin;
      float prob_hit, prob_miss;
      float free_threshold, occupancy_threshold;
      float min_clamp, max_clamp;
      unsigned int block_size;
      bool lock_at_max_clamp;
    } params_t;
  }  // namespace grid2d
}  // namespace map_utils