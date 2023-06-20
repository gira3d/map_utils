#ifndef MAP_UTILS_GRID_3D_KDTREE_H
#define MAP_UTILS_GRID_3D_KDTREE_H

#include <limits>
#include <kdtree/kdtree.h>
#include "Grid3D.h"

namespace map_utils
{
  namespace grid3d
  {
    namespace kdtree
    {
      typedef struct KDTree
      {
        typedef boost::shared_ptr<KDTree> Ptr;
        typedef boost::shared_ptr<const KDTree> ConstPtr;

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
          tree = kd_create(3);
        }

        inline void destroy()
        {
          if (tree != 0)
            kd_free(tree);
          tree = 0;
        }

        inline void insert(const Point& p)
        {
          float pos[3] = {p.x, p.y, p.z};
          kd_insertf(tree, pos, 0);
        }

        inline bool nearest(const Point& in, Point& out) const
        {
          float pos[3] = {in.x, in.y, in.z};

          kdres* set = kd_nearestf(tree, pos);
          // Handle the case where the tree is empty
          if (!set)
            return false;

          int count = kd_res_size(set);
          if (count == 0)
          {
            fprintf(stderr, "[Grid3DKDTree]: nearest result is invalid\n");
            kd_res_free(set);
            return false;
          }

          float np[3];
          kd_res_itemf(set, np);
          out.x = np[0];
          out.y = np[1];
          out.z = np[2];
          kd_res_free(set);

          return true;
        }

      } kdtree_t;

      // Creates a kd tree from grid cells matching the input state in a given
      // bounding box.
      //
      // Grid values are assumed to be undefined outside the bounds of the grid
      // so the input must be entirely inside the grid bounding box.
      inline bool createKDTree(const Grid3D& grid,
                               const BoundingBox& bbx,
                               const Grid3D::cell_state_t& s,
                               KDTree& out)
      {
        if (!grid.getBBX().inQ(bbx))
        {
          fprintf(stderr, "[Grid3DKDTree]: input bbx not in grid \n");
          return false;
        }

        if (!bbx.isNonEmpty())
        {
          fprintf(stderr, "[Grid3DKDTree]: input bbx is empty\n");
        }

        out.destroy();
        out.create();

        Cell cl = grid.w2c(bbx.min);
        Cell cu = grid.w2c(bbx.max);

        for (unsigned int i = cl.row; i < cu.row; i++)
          for (unsigned int j = cl.col; j < cu.col; j++)
            for (unsigned int k = cl.slice; k < cu.slice; k++)
              if (grid.getCellState(Cell(i, j, k)) == s)
                out.insert(grid.c2w(Cell(i, j, k)));

        out.initialized = true;
        out.load_time = grid.load_time;

        return true;
      }

      inline void destroyKDTree(KDTree& kd)
      {
        kd.destroy();
      }

      inline float nearest(const KDTree& kd, const Point& p)
      {
        Point pn;
        bool ret = kd.nearest(p, pn);
        return ret ? sqrtf((pn.x - p.x)*(pn.x - p.x) +
                           (pn.y - p.y)*(pn.y - p.y) +
                           (pn.z - p.z)*(pn.z - p.z)) : std::numeric_limits<float>::infinity();
      }

    }
  }
}
#endif
