#ifndef MAP_UTILS_GRID_3D_BOUNDING_BOX_H
#define MAP_UTILS_GRID_3D_BOUNDING_BOX_H

#include <string>
#include <iostream>
#include <stdio.h>
#include <cfloat>
#include <vector>
#include <utility>

#include "Grid3DPoint.h"

// Bounding box semantics:
//
// The bounding box is defined as the set produced by the product of closed
// intervals [min, max] for each of x, y, and sometimes z.
//
// As a corollary, a bounding box is *empty* iff max < min for any axis.
//
// If a bounding box is *non-empty* and max == min along some axis, then the
// bounding box has zero volume. *Therefore, being empty and having not volume
// are distinct.*
//
// Because bounding boxes are defined based on closed intervals, all operations
// can be assumed to also use closed intervals. Operations on the interior (or
// which would otherwise treat the bounding box as an open set) should be
// clearly specified.
//
// Additionally, some operations may be defined using tolerances. Tolerances are
// all optional and default to zero.
//
// Users may also wish to use intervals that are open on the upper end as in
// [min, max). This has the useful property that adjacent bounding boxes
// e.g. on [a, b) union [b, c) don't have holes, similarly as for integer
// ranges. As useful as this may seem, this is not the default behavior because
// * Tiling of maps and bounding boxes is not common in the current use case
// * To ensure that the bounding box defined as containing a set of points does,
//   in fact, contain those points.
// As such, single-ended behavior is not yet supported but may be added as an
// option in the future.
//
// Bounding boxes describe sets so it is natural to consider defining different
// set operations. However, bounding boxes are not closed under set operations
// e.g. the set-union of two bounding boxes is not necessarily a bounding box.
// Instead, we define analogues of set operations by lifting. To lift a set
// operation, take the bounding box of the result of applying the given set
// operation to (the sets associated with) the bounding boxes given as inputs.

#define CMP(x, y) \
        (fabsf(x - y) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))
namespace map_utils
{
  namespace grid3d
  {

    /*
     * Ray struct, for use with the optimized ray-box intersection test
     * described in:
     *
     *      Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley
     *      "An Efficient and Robust Ray-Box Intersection Algorithm"
     *      Journal of graphics tools, 10(1):49-54, 2005
     */
    typedef struct Ray
    {
      Ray() {}
      Ray(Point st, Point end)
      {
	origin = st;
	direction = (end-st).normalize();
	inv_direction = Point(1/direction.x, 1/direction.y, 1/direction.z);
	sign[0] = (inv_direction.x < 0);
	sign[1] = (inv_direction.y < 0);
	sign[2] = (inv_direction.z < 0);
      }

      Ray(const Ray &r)
      {
	origin = r.origin;
	direction =r.origin;
	inv_direction = r.inv_direction;
	sign[0] = r.sign[0]; sign[1] = r.sign[1]; sign[2] = r.sign[2];
      }

      Point origin;
      Point direction;
      Point inv_direction;
      int sign[3];
    } ray_t;

    typedef struct BoundingBox
    {
      typedef boost::shared_ptr<BoundingBox> Ptr;
      typedef boost::shared_ptr<const BoundingBox> ConstPtr;

      // Initialize bounding boxes as empty
      Point min = Point(1,1,1);
      Point max = Point(-1,-1,-1);

      BoundingBox() { }

      // Construct a bounding box either directly via the min and max entries or
      // using the bounding box of the two points (default).
      //
      // The behavior of the initialization methods is as follows:
      //
      // DIRECT: Sets this->max = p1 and this->min = p2.
      //
      // This is appropriate when p1 and p2 explicitly match the min and max
      // fields. This means that the bounding box will be empty if p2 < p1 along
      // any axis.
      //
      // UNION: Construct the bounding box to contain the two input points
      // (e.g. the bounding box union of the set of each point)
      // Effectively, this swaps entries of p1 and p2 so that this->max contains
      // the greater and this->min contains the lesser.
      //
      // This is equivalent to the original behavior of this constructor for
      // BoundingBox(p1,p2) before adding the "method" option.
      //
      // UNION_WARNEMPTY: *default* This implements the behavior for UNION
      // (which was the original behavior for BoudingBox(p1,p2)) but prints a
      // warning if p2 < p1 for any axis.
      //
      // Note: if you are coming here after seeing that warning, please specify
      // either the DIRECT or UNION behavior as the behavior of this constructor
      // is otherwise ambiguous.
      static const int DIRECT = 0;
      static const int UNION = 1;
      static const int UNION_WARNEMPTY = 2;
      BoundingBox(const Point& p1, const Point& p2,
                  int method = UNION_WARNEMPTY)
      {
        switch(method)
        {
          case UNION :
            {
              // Output the bounding box containing the union of p1 and p2
              setUnionOfPoints(p1, p2);
              break;
            }
          case DIRECT :
            {
              // Set min/max fields directly
              setMinMax(p1, p2);
              break;
            }
          case UNION_WARNEMPTY :
            {
              // If the bounding box for DIRECT is empty, the UNION and DIRECT
              // methods would produce different outputs.
              //
              // Warn the user of this ambiguity.
              //
              // Then implement the default "union" behavior.
              auto b = BoundingBox(p1, p2, DIRECT);
              if (!b.isNonEmpty())
              {
                std::cerr << "Warning in constructor BoundingBox(p1, p2)!\n"
                          << "  p2 < p1 along some axis, which produces ambiguous behavior!\n"
                          << "\n"
                          << "  Please specify the construction method in the form:\n"
                          << "  BoundingBox(p1, p2, method)\n"
                          << "  where method = DIRECT or UNION\n"
                          << "  (see comments in code for descriptions of construction methods)";
              }

              setUnionOfPoints(p1, p2);
              break;
            }
          default :
            {
              std::cerr << method
                << " is not a proper method for construction of a bounding box"
                << std::endl;

              throw;
            }
        }
      }

      // Produce a minimal bounding box containing the union of the input
      // points.
      //
      // Note that this does not have a method switch because there would be no
      // reason to use this method to set the min and max directly.
      BoundingBox(const std::vector<Point>& points)
      {
        setUnionOfPoints(points);
      }

      BoundingBox(const BoundingBox& b) : min(b.min), max(b.max) { }

      //returns a bounding box centered around a sphere of a given radius
      static BoundingBox fromRadius(double radius)
      {
        return BoundingBox(Point(-radius, -radius, -radius),
                           Point(radius, radius, radius));
      }

      // instantiate a bounding box around a ball with given center and radius
      static BoundingBox ball(Point center, double radius)
      {
        return BoundingBox(center + Point(-radius, -radius, -radius),
                           center + Point(radius, radius, radius));
      }

      // Convenience setters

      // Explicitly set both the min and max fields.
      //
      // The bounding box will be "empty" if max < min along any axis.
      inline void setMinMax(const Point& min, const Point& max)
      {
        this->min = min;
        this->max = max;
      }

      // set membership

      inline bool inQ(const Point& in) const
      {
        return (in.x >= min.x) && (in.y >= min.y) && (in.z >= min.z) &&
               (in.x <= max.x) && (in.y <= max.y) && (in.z <= max.z);
      }

      // returns true if the input bounding box is a subset of this
      inline bool inQ(const BoundingBox& in) const
      {
        return (inQ(in.min) && inQ(in.max)) || !in.isNonEmpty();
      }

      // Returns true iff the bounding box is non-empty
      // i.e. is not the empty set (but may correspond to a set of zero volume)
      //
      // According to the definition of the bounding box, this is true iff for
      // all of the x/y/z ranges max >= min.
      //
      // Historical note: the "hasZeroVolume" method used to be named "isEmpty."
      // As such, we will not be adding isEmpty until that stabilizes to prevent
      // aliasing the names.
      inline bool isNonEmpty() const
      {
        return max.x >= min.x &&
               max.y >= min.y &&
               max.z >= max.z;
      }

      // Begin in-place set operations

      // set the bounding box to be the empty set
      inline void setEmpty()
      {
        min = Point(1, 1, 1);
        max = Point(-1, -1, -1);
      }

      // Set the bounding box to the bounding box containing only "p"
      inline void setPoint(const Point& p)
      {
        min = p;
        max = p;
      }

      // Sets the bounding box to be the union of the *current* bounding box and
      // a point
      //
      // Note: "union" is a reserved word.
      //
      // Warning: Behavior differs from setUnionOfPoints which resets the
      // bounding box.
      inline void setUnionWithPoint(const Point& p)
      {
        // This code sets the bounding box to the union with a point while
        // handling the special case of an initially empty bounding box.
        //
        // Note: intuitive max/min operations in setUnionWithPointNonEmpty do
        // not apply when the bounding box is empty (max < min).
        //
        // For an example, consider an empty bounding box (max < min) and an
        // input point p > min. The intuitive approach returns [min, p], but the
        // correct output is [p, p].
        if(isNonEmpty())
        {
          setUnionWithPointNonEmpty(p);
        }
        else
        {
          setPoint(p);
        }
      }

      // Set the bounding box to be the bounding box containing the current
      // volume plus the given point.
      //
      // This is largely a helper function for setUnionOfPoints to incrementally
      // update a bounding box.
      //
      // Also, see the description of setUnionWithPoint.
      //
      // Warning: Do not apply this method unless this bounding box is known to
      // be non-empty.
      inline void setUnionWithPointNonEmpty(const Point& p)
      {
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);

        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
      }

      // Set to the bounding box containing the union of two points
      // (discarding whatever the bounding box is currently set to)
      //
      // First, set the bounding box to contain exactly the first point.
      // Then, add the second.
      // (this avoids issues related to empty bounding boxes)
      inline void setUnionOfPoints(const Point& p1, const Point& p2)
      {
        setPoint(p1);
        setUnionWithPointNonEmpty(p2);
      }

      // Set to the bounding box containing the union of a vector of points
      // (discarding whatever the bounding box is currently set to)
      //
      // If the input vector is empty, set the bounding box to be the empty set.
      //
      // Else, construct the bounding box incrementally
      inline void setUnionOfPoints(const std::vector<Point>& points)
      {
        // If the set of points is empty, the bounding box should be empty.
        if(points.empty())
        {
          setEmpty();
        }
        // There is at least one point
        else
        {
          setPoint(points.front());

          for(auto& point : points)
          {
            setUnionWithPointNonEmpty(point);
          }
        }
      }

      // Minkowski sums and and Pontryagin differences

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

      inline bool equals(const BoundingBox& that, const float tol = 0.0f) const
      {
        return ((fabsf(this->min.x - that.min.x) < tol) &&
                (fabsf(this->min.y - that.min.y) < tol) &&
                (fabsf(this->min.z - that.min.z) < tol) &&
                (fabsf(this->max.x - that.max.x) < tol) &&
                (fabsf(this->max.y - that.max.y) < tol) &&
                (fabsf(this->max.z - that.max.z) < tol));
      }

      inline void clip(BoundingBox& bbx) const
      {
        bbx.min.x = fmaxf(bbx.min.x, min.x);
        bbx.min.y = fmaxf(bbx.min.y, min.y);
        bbx.min.z = fmaxf(bbx.min.z, min.z);

        bbx.max.x = fminf(bbx.max.x, max.x);
        bbx.max.y = fminf(bbx.max.y, max.y);
        bbx.max.z = fminf(bbx.max.z, max.z);
      }

      inline bool raycast(const Point& origin, const Point& dir, Point& intersection)
      {

        // Any component of direction could be 0!
        // Address this by using a small number, close to
        // 0 in case any of directions components are 0
        float t1 = (min.x - origin.x) / (CMP(dir.x, 0.0f) ? 0.00001f : dir.x);
        float t2 = (max.x - origin.x) / (CMP(dir.x, 0.0f) ? 0.00001f : dir.x);
        float t3 = (min.y - origin.y) / (CMP(dir.y, 0.0f) ? 0.00001f : dir.y);
        float t4 = (max.y - origin.y) / (CMP(dir.y, 0.0f) ? 0.00001f : dir.y);
        float t5 = (min.z - origin.z) / (CMP(dir.z, 0.0f) ? 0.00001f : dir.z);
        float t6 = (max.z - origin.z) / (CMP(dir.z, 0.0f) ? 0.00001f : dir.z);

        float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
        float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

        // if tmax < 0, ray is intersecting AABB
        // but entire AABB is behind it's origin
        if (tmax < 0 || tmin > tmax)
          return false;

        float t_result = tmin < 0.f ? tmax : tmin;
        t_result *= (inQ(origin) ? 0.999 : 1.0001);
        intersection = origin + t_result * dir;

        return true;
      }

      void print(const std::string& prefix = std::string()) const
      {
        if (!prefix.empty())
          printf("%s: min(x, y, z), max(x, y, z) = (%f, %f, %f), (%f, %f, %f)\n",
                 prefix.c_str(), min.x, min.y, min.z, max.x, max.y, max.z);
        else
          printf("min(x, y, z), max(x, y, z) = (%f, %f, %f), (%f, %f, %f)\n",
                 min.x, min.y, min.z, max.x, max.y, max.z);
      }

      /* returns the intersection of two bounding boxes */
      inline BoundingBox intersect(const BoundingBox &bbx)
      {
        BoundingBox b(bbx);
        clip(b);
        return b;
      }

      // Returns volume of the bounding box
      //
      // The bounding box has zero volume if, for any axis, max <= min
      inline float volume()
      {
        if(hasZeroVolume())
        {
          return 0.0f;
        }

        auto diff = max - min;

        return diff.x*diff.y*diff.z;
      }

      // Returns true if the lengths of all dimensions are greater than
      // length_tol
      inline bool hasZeroVolume(float length_tol=0.0f)
      {
        auto diff = max - min;

        return diff.x <= length_tol ||
               diff.y <= length_tol ||
               diff.z <= length_tol;
      }

      // Given *this* and *that* bounding boxes, the set difference returns the set of
      // elements in *this* but not in *that* as three non-overlapping bounding boxes
      // Limitation: *this* must intersect with *that* before calling this function
      inline void setDifference(const BoundingBox& that, std::vector<BoundingBox>& bbxs)
      {
        typedef std::pair<float, float> Pair;

        BoundingBox bbx2 = BoundingBox(min, max, DIRECT);
        BoundingBox bbx3 = intersect(that);

        Point min_, max_;

        bbxs.clear();
        bbxs.resize(3);

        Pair x, y, z;

        // handles axis that is not flush with intersection bbx
        auto case1 = [](Pair a, Pair b)
          {return (a.first < b.first) ? Pair(a.first, b.first): Pair(b.second, a.second);};

        // clips to dimensions of the intersection bbx
        auto case2 = [](Pair a, Pair b)
          {return (a.first < b.first) ? Pair(a.first, b.second): Pair(b.first, a.second);};

        // First take care of the nonoverlapping bbx in x-direction
        x = case1({bbx2.min.x, bbx2.max.x}, {bbx3.min.x, bbx3.max.x});
        y = case2({bbx2.min.y, bbx2.max.y}, {bbx3.min.y, bbx3.max.y});
        z = case2({bbx2.min.z, bbx2.max.z}, {bbx3.min.z, bbx3.max.z});
        min_ = Point(x.first, y.first, z.first);
        max_ = Point(x.second, y.second, z.second);
        bbxs[0] = BoundingBox(min_, max_, DIRECT);

        // Second, consider the nonoverlapping bbx in y-direction.
        // We don't want to double-count the x-direction, so fix x to the bounds of intersected bbx
        x = {bbx3.min.x, bbx3.max.x};
        y = case1({bbx2.min.y, bbx2.max.y}, {bbx3.min.y, bbx3.max.y});
        min_ = Point(x.first, y.first, z.first);
        max_ = Point(x.second, y.second, z.second);
        bbxs[1] = BoundingBox(min_, max_, DIRECT);

        // We don't want to double-count x or y, so fix them both.
        y = {bbx3.min.y, bbx3.max.y};
        z = case1({bbx2.min.z, bbx2.max.z}, {bbx3.min.z, bbx3.max.z});
        min_ = Point(x.first, y.first, z.first);
        max_ = Point(x.second, y.second, z.second);
        bbxs[2] = BoundingBox(min_, max_, DIRECT);
      }

      /*
       * Ray-box intersection using IEEE numerical properties to ensure that the
       * test is both robust and efficient, as described in:
       *
       *      Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley
       *      "An Efficient and Robust Ray-Box Intersection Algorithm"
       *      Journal of graphics tools, 10(1):49-54, 2005
       *
       * @param[in] ray_t the ray between sensor origin and endpoint
       * @param[in] t0 the minimum valid intersection interval
       * @param[in] t1 the maximum valid intersection interval
       * @param[out] tmin the smaller distance between sensor origin and bbx
       * @param[out] tmax the larger distance between sensor origin and bbx
       *
       * Returns true if there is at least one intersection point between
       * ray.origin + t0 * ray.direction and ray.origin + t1 * ray.direction
       */
      inline bool rayIntersect(const ray_t& r, float t0, float t1, float& tmin, float& tmax) const
      {
	Point parameters[2] = {min, max};
	float tymin, tymax, tzmin, tzmax;

	tmin = (parameters[r.sign[0]].x - r.origin.x) * r.inv_direction.x;
	tmax = (parameters[1-r.sign[0]].x - r.origin.x) * r.inv_direction.x;
	tymin = (parameters[r.sign[1]].y - r.origin.y) * r.inv_direction.y;
	tymax = (parameters[1-r.sign[1]].y - r.origin.y) * r.inv_direction.y;
	if ( (tmin > tymax) || (tymin > tmax) )
	  return false;
	if (tymin > tmin)
	  tmin = tymin;
	if (tymax < tmax)
	  tmax = tymax;
	tzmin = (parameters[r.sign[2]].z - r.origin.z) * r.inv_direction.z;
	tzmax = (parameters[1-r.sign[2]].z - r.origin.z) * r.inv_direction.z;
	if ( (tmin > tzmax) || (tzmin > tmax) )
	  return false;
	if (tzmin > tmin)
	  tmin = tzmin;
	if (tzmax < tmax)
	  tmax = tzmax;
	return ( (tmin < t1) && (tmax > t0) );
      }
    } bbx_t;
  }
}
#endif
