#pragma once

namespace map_utils
{
  namespace grid3d
  {
    typedef struct Parameters
    {
      typedef boost::shared_ptr<Parameters> Ptr;
      typedef boost::shared_ptr<const Parameters> ConstPtr;

      unsigned int width;
      unsigned int height;
      unsigned int depth;

      float resolution;
      Point origin;
      float prob_hit, prob_miss;
      float free_threshold, occupancy_threshold;
      float min_clamp, max_clamp;
      unsigned int block_size;
      bool track_changes;
      bool lock_at_max_clamp;
    } params_t;
  }  // namespace grid3d
}  // namespace map_utils