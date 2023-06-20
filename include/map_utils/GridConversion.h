#ifndef MAP_UTILS_GRID_CONVERSION_H
#define MAP_UTILS_GRID_CONVERSION_H

#include "Grid2D.h"
#include "Grid3D.h"

namespace map_utils
{
  namespace conversion
  {
    inline grid2d::Grid2D::Ptr createGrid2DSlice(const grid3d::Grid3D& in,
                                                 const grid3d::BoundingBox& b)
    {
      grid3d::bbx_t bbx(b);
      in.getBBX().clip(bbx);

      grid3d::Cell cl = in.w2c(bbx.min);
      grid3d::Cell cu = in.w2c(bbx.max);

      // Remove the buffer added by getBBX if on boundary
      if (cu.row == in.height) cu.row--;
      if (cu.col == in.width) cu.col--;
      if (cu.slice == in.depth) cu.slice--;

      unsigned int dr = cu.row - cl.row + 1;
      unsigned int dc = cu.col - cl.col + 1;
      unsigned int ds = cu.slice - cl.slice + 1;

      grid3d::Point origin_3d = in.c2w(cl);
      grid2d::Point origin_2d(origin_3d.x, origin_3d.y);

      grid2d::Grid2D::Ptr out =
        grid2d::Grid2D::Ptr(new grid2d::Grid2D(dc, dr, in.resolution, origin_2d));
      out->load_time = in.load_time;
      out->log_odds_hit = in.log_odds_hit;
      out->log_odds_miss = in.log_odds_miss;
      out->free_threshold = in.free_threshold;
      out->occupancy_threshold = in.occupancy_threshold;
      out->min_clamp = in.min_clamp;
      out->max_clamp = in.max_clamp;

      for (unsigned int r = 0; r < dr; r++)
        for (unsigned int c = 0; c < dc; c++)
        {
          grid3d::Grid3D::cell_state_t state = grid3d::Grid3D::UNKNOWN;
          for (unsigned int s = 0; s < ds; s++)
          {
            switch (in.getCellState(grid3d::Cell(cl.row + r, cl.col + c, cl.slice + s)))
            {
              case grid3d::Grid3D::UNKNOWN: break;
              case grid3d::Grid3D::FREE:
              {
                if (state != grid3d::Grid3D::OCCUPIED)
                  state = grid3d::Grid3D::FREE;
                break;
              }
              case grid3d::Grid3D::OCCUPIED:
              {
                state = grid3d::Grid3D::OCCUPIED;
                break;
              }
            }
          }

          switch (state)
          {
            case grid3d::Grid3D::UNKNOWN:
            {
              out->set(grid2d::Cell(r, c), 0.0f);
              break;
            }
            case grid3d::Grid3D::FREE:
            {
              out->set(grid2d::Cell(r, c), -4.5f); // ~0.01 prob
              break;
            }
            case grid3d::Grid3D::OCCUPIED:
            {
              out->set(grid2d::Cell(r, c), 4.5f); // ~0.99 prob
              break;
            }
          }
        }

      return out;
    }
  }
}
#endif
