#ifndef MAP_UTILS_GRID_3D_POINT_H
#define MAP_UTILS_GRID_3D_POINT_H

#include <cmath>
#include <string>
#include <boost/shared_ptr.hpp>

namespace map_utils
{
  namespace grid3d
  {
    typedef struct Point
    {
      typedef boost::shared_ptr<Point> Ptr;
      typedef boost::shared_ptr<const Point> ConstPtr;

      float x, y, z;
      Point() : x(0.0f), y(0.0f), z(0.0f) {}
      Point(const Point& p) : x(p.x), y(p.y), z(p.z) {}
      Point(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

      inline Point operator+(const Point& rhs) const
      {
        return Point(x + rhs.x, y + rhs.y, z + rhs.z);
      }

      inline Point operator-(const Point& rhs) const
      {
        return Point(x - rhs.x, y - rhs.y, z - rhs.z);
      }

      inline Point operator*(float rhs) const
      {
        return Point(x*rhs, y*rhs, z*rhs);
      }

      inline Point operator/(float rhs) const
      {
        return Point(x/rhs, y/rhs, z/rhs);
      }

      inline Point operator+=(const Point& rhs)
      {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
      }

      inline Point operator-=(const Point& rhs)
      {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
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
                (fabsf(y - that.y) < tol) &&
                (fabsf(z - that.z) < tol));
      }

      inline Point normalize()
      {
        float norm = std::sqrt(x*x + y*y + z*z);
        return norm < 0.0001f ? *this : (*this / norm);
      }

      void print(const std::string& prefix = std::string()) const
      {
        if (!prefix.empty())
          printf("%s: (x, y, z) = (%f, %f, %f)\n",
                 prefix.c_str(), x, y, z);
        else
          printf("(x, y, z) = (%f, %f, %f)\n", x, y, z);
      }
    } point_t;

    inline Point operator*(float lhs, const Point& rhs)
    {
      return Point(lhs*rhs.x, lhs*rhs.y, lhs*rhs.z);
    }
  }
}
#endif
