#ifndef MAP_UTILS_GRID_3D_CELL_H
#define MAP_UTILS_GRID_3D_CELL_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <stdio.h>

namespace map_utils
{
  namespace grid3d
  {
    typedef struct Cell
    {
      typedef boost::shared_ptr<Cell> Ptr;
      typedef boost::shared_ptr<const Cell> ConstPtr;

      unsigned int row, col, slice;
      Cell() : row(0), col(0), slice(0) {}
      Cell(unsigned int r, unsigned int c, unsigned int s) : row(r), col(c), slice(s) {}
      inline bool operator==(const Cell& other) const
      {
        return ((row == other.row) && (col == other.col) && (slice == other.slice));
      }

      inline Cell operator-(const Cell& rhs) const
      {
        return Cell(row - rhs.row, col - rhs.col, slice - rhs.slice);
      }

      inline Cell operator+(const Cell& rhs) const
      {
        return Cell(row + rhs.row, col + rhs.col, slice + rhs.slice);
      }

      void print(const std::string& prefix = std::string()) const
      {
        if (!prefix.empty())
          printf("%s: (r, c, s) = (%u, %u, %u)\n",
                 prefix.c_str(), row, col, slice);
        else
          printf("(r, c, s) = (%u, %u, %u)\n", row, col, slice);
      }
    } cell_t;
  }
}
#endif
