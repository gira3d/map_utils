#ifndef MAP_UTILS_GRID_2D_BOUNDING_BOX_H
#define MAP_UTILS_GRID_2D_BOUNDING_BOX_H

#include "Grid2DPoint.h"

#include <stdio.h>

namespace map_utils
{
  namespace grid2d
  {
    typedef struct BoundingBox
    {
      typedef boost::shared_ptr<BoundingBox> Ptr;
      typedef boost::shared_ptr<const BoundingBox> ConstPtr;

      Point min, max;

      BoundingBox() { }
      BoundingBox(const Point& p1, const Point& p2)
      {
        if (p1.x < p2.x)
        {
          min.x = p1.x;
          max.x = p2.x;
        }
        else
        {
          min.x = p2.x;
          max.x = p1.x;
        }

        if (p1.y < p2.y)
        {
          min.y = p1.y;
          max.y = p2.y;
        }
        else
        {
          min.y = p2.y;
          max.y = p1.y;
        }
      }

      BoundingBox(const BoundingBox& b) : min(b.min), max(b.max) { }

      inline bool inQ(const Point& in) const
      {
        return ((in.x > min.x) && (in.y > min.y) && (in.x < max.x) && (in.y < max.y));
      }

      inline bool inQ(const BoundingBox& in) const
      {
        return ((inQ(in.min) && inQ(in.max)) || equals(in));
      }

      inline BoundingBox operator+(const BoundingBox& rhs) const
      {
        return BoundingBox(min + rhs.min, max + rhs.max);
      }

      inline BoundingBox operator-(const BoundingBox& rhs) const
      {
        return BoundingBox(min - rhs.min, max - rhs.max);
      }

      inline BoundingBox operator+=(const BoundingBox& rhs)
      {
        min += rhs.min;
        max += rhs.max;
        return *this;
      }

      inline BoundingBox operator-=(const BoundingBox& rhs)
      {
        min -= rhs.min;
        max -= rhs.max;
        return *this;
      }

      inline bool operator==(const BoundingBox& that) const
      {
        return this->equals(that);
      }

      inline bool operator!=(const BoundingBox& that) const
      {
        return !this->equals(that);
      }

      inline bool equals(const BoundingBox& that, const float tol = 1e-8f) const
      {
        return ((fabsf(this->min.x - that.min.x) < tol) &&
                (fabsf(this->min.y - that.min.y) < tol) &&
                (fabsf(this->max.x - that.max.x) < tol) &&
                (fabsf(this->max.y - that.max.y) < tol));
      }

      inline void clip(BoundingBox& bbx) const
      {
        bbx.min.x = fmaxf(bbx.min.x, min.x);
        bbx.min.y = fmaxf(bbx.min.y, min.y);
        bbx.max.x = fminf(bbx.max.x, max.x);
        bbx.max.y = fminf(bbx.max.y, max.y);

        // For degenerates
        bbx.min.x = fminf(bbx.max.x, bbx.min.x);
        bbx.min.y = fminf(bbx.max.y, bbx.min.y);
        bbx.max.x = fmaxf(bbx.max.x, bbx.min.x);
        bbx.max.y = fmaxf(bbx.max.y, bbx.min.y);
      }

      void print(const std::string& prefix = std::string()) const
      {
        if (!prefix.empty())
          printf("%s: min(x, y), max(x, y) = (%f, %f), (%f, %f)\n",
                 prefix.c_str(), min.x, min.y, max.x, max.y);
        else
          printf("min(x, y), max(x, y) = (%f, %f), (%f, %f)\n",
                 min.x, min.y, max.x, max.y);
      }

      /* returns the intersection of two bounding boxes */
      inline BoundingBox intersect(const BoundingBox &bbx)
      {
        BoundingBox b(bbx);
        clip(b);
        return b;
      }

      // Returns true if the lengths of all dimensions are greater than
      // length_tol
      inline bool isEmpty(float length_tol=0.0f)
      {
        auto diff = max - min;

        return diff.x <= length_tol ||
               diff.y <= length_tol;
      }
    } bbx_t;
  }
}
#endif
