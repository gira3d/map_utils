#ifndef MAP_UTILS_GRID_2D_KDTREE_H
#define MAP_UTILS_GRID_2D_KDTREE_H

#include <limits>
#include <kdtree/kdtree.h>
#include <stdio.h>

#include "Grid2D.h"

namespace map_utils
{
  namespace grid2d
  {
    namespace kdtree
    {
      typedef struct KDTree
      {
        double load_time;
        bool initialized;
        struct kdtree* tree;

        KDTree() : load_time(0), initialized(false), tree(0) {}
        ~KDTree()
        {
          destroy();
        }

        inline void create()
        {
          tree = kd_create(2);
        }

        inline void destroy()
        {
          if (tree != 0)
            kd_free(tree);
          tree = 0;
        }

        inline void insert(const Point& p)
        {
          float pos[2] = {p.x, p.y};
          kd_insertf(tree, pos, 0);
        }

        inline bool nearest(const Point& in, Point& out) const
        {
          float pos[2] = {in.x, in.y};

          kdres* set = kd_nearestf(tree, pos);
          int count = kd_res_size(set);
          if (count == 0)
          {
            fprintf(stderr, "[Grid2DKDTree]: nearest result is invalid\n");
            kd_res_free(set);
            return false;
          }

          float np[2];
          kd_res_itemf(set, np);
          out.x = np[0];
          out.y = np[1];
          kd_res_free(set);

          return true;
        }

      } kdtree_t;

      inline void createKDTree(const Grid2D& grid,
                               const BoundingBox& bbx,
                               const Grid2D::cell_state_t& s,
                               KDTree& out)
      {
        if ((!grid.inQ(bbx.min)) || (!grid.inQ(bbx.max)))
        {
          fprintf(stderr, "[Grid2DKDtree]: bbx not in grid\n");
          return;
        }

        out.destroy();
        out.create();

        Cell cl = grid.w2c(bbx.min);
        Cell cu = grid.w2c(bbx.max);

        for (unsigned int i = cl.row; i < cu.row; i++)
          for (unsigned int j = cl.col; j < cu.col; j++)
            if (grid.getCellState(Cell(i, j)) == s)
              out.insert(grid.c2w(Cell(i, j)));

        out.initialized = true;
        out.load_time = grid.load_time;
      }

      inline void destroyKDTree(KDTree& kd)
      {
        kd.destroy();
      }

      inline float nearest(const KDTree& kd, const Point& p)
      {
        Point pn;
        bool ret = kd.nearest(p, pn);
        return ret ? hypotf(pn.x - p.x, pn.y - p.y) : std::numeric_limits<float>::infinity();
      }

    }
  }
}
#endif
