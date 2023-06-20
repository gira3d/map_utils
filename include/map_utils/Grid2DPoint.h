#ifndef MAP_UTILS_GRID_2D_POINT_H
#define MAP_UTILS_GRID_2D_POINT_H

#include <cmath>
#include <string>
#include <boost/shared_ptr.hpp>
#include <stdio.h>

namespace map_utils
{
  namespace grid2d
  {
    typedef struct Point
    {
      typedef boost::shared_ptr<Point> Ptr;
      typedef boost::shared_ptr<const Point> ConstPtr;

      float x, y;
      Point() : x(0.0f), y(0.0f) {}
      Point(const Point& p) : x(p.x), y(p.y) {}
      Point(float x_, float y_) : x(x_), y(y_) {}

      inline Point operator+(const Point& rhs) const
      {
        return Point(x + rhs.x, y + rhs.y);
      }

      inline Point operator-(const Point& rhs) const
      {
        return Point(x - rhs.x, y - rhs.y);
      }

      inline Point operator*(float rhs) const
      {
        return Point(x*rhs, y*rhs);
      }

      inline Point operator/(float rhs) const
      {
        return Point(x/rhs, y/rhs);
      }

      inline Point operator+=(const Point& rhs)
      {
        x += rhs.x;
        y += rhs.y;
        return *this;
      }

      inline Point operator-=(const Point& rhs)
      {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
      }

      inline bool operator==(const Point& that) const
      {
        return this->equals(that);
      }

      inline bool operator!=(const Point& that) const
      {
        return !this->equals(that);
      }

      inline bool equals(const Point& that, const float tol = 1e-8f) const
      {
        return ((fabsf(x - that.x) < tol) &&
                (fabsf(y - that.y) < tol));
      }

      void print(const std::string& prefix = std::string()) const
      {
        if (!prefix.empty())
          printf("%s: (x, y) = (%f, %f)\n",
                 prefix.c_str(), x, y);
        else
          printf("(x, y) = (%f, %f)\n", x, y);
      }
    } point_t;

    inline Point operator*(float lhs, const Point& rhs)
    {
      return Point(lhs*rhs.x, lhs*rhs.y);
    }
  }
}
#endif
