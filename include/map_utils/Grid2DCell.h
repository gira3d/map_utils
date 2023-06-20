#ifndef MAP_UTILS_GRID_2D_CELL_H
#define MAP_UTILS_GRID_2D_CELL_H

#include <boost/shared_ptr.hpp>
#include <string>
#include <stdio.h>

namespace map_utils
{
  namespace grid2d
  {
    typedef struct Cell
    {
      typedef boost::shared_ptr<Cell> Ptr;
      typedef boost::shared_ptr<const Cell> ConstPtr;

      unsigned int row, col;
      Cell() : row(0), col(0) {}
      Cell(unsigned int r, unsigned int c) : row(r), col(c) {}
      bool operator==(const Cell& other) const
      {
        return ((row == other.row) && (col == other.col));
      }

      void print(const std::string& prefix = std::string()) const
      {
        if (!prefix.empty())
          printf("%s: (r, c) = (%u, %u)\n",
                 prefix.c_str(), row, col);
        else
          printf("(r, c) = (%u, %u)\n", row, col);
      }
    } cell_t;
  }
}
#endif
