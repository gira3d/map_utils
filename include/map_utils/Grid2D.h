#pragma once

#include <vector>
#include <cmath>
#include <stdio.h>

#include "Grid2DCell.h"
#include "Grid2DCellValue.h"
#include "Grid2DPoint.h"
#include "Grid2DBoundingBox.h"
#include "Grid2DParameters.h"

namespace map_utils
{
  namespace grid2d
  {
    template <typename CELL_VALUE = CellValue>
    struct Grid2D
    {
      typedef boost::shared_ptr<Grid2D> Ptr;
      typedef boost::shared_ptr<const Grid2D> ConstPtr;

      typedef enum
      {
        FREE,
        UNKNOWN,
        OCCUPIED
      } cell_state_t;

      params_t params;
      std::vector<CELL_VALUE> data;
      unsigned int width, height;
      float resolution;
      double load_time = 0.0;
      Point origin;
      float log_odds_hit, log_odds_miss;
      float free_threshold, occupancy_threshold;
      float min_clamp, max_clamp;
      unsigned int block_size;
      bool lock_at_max_clamp;

      Grid2D() : width(0), height(0), resolution(0.0f), lock_at_max_clamp(false)
      {
      }
      Grid2D(unsigned int w, unsigned int h)
        : width(w)
        , height(h)
        , resolution(0.0f)
        , block_size(1)
        , lock_at_max_clamp(false)
      {
        data.resize(width * height, CELL_VALUE{});
      }

      Grid2D(unsigned int w, unsigned int h, float r)
        : width(w)
        , height(h)
        , resolution(r)
        , block_size(1)
        , lock_at_max_clamp(false)
      {
        data.resize(width * height, CELL_VALUE{});
      }

      Grid2D(unsigned int w, unsigned int h, float r, const Point& p)
        : width(w)
        , height(h)
        , resolution(r)
        , origin(p)
        , block_size(1)
        , lock_at_max_clamp(false)
      {
        data.resize(width * height, CELL_VALUE{});
      }

      Grid2D(params_t p)
      {
        params = p;

        width = p.width;
        height = p.height;
        origin = p.origin;
        resolution = p.resolution;

        data.resize(width * height, CELL_VALUE{});

        setLogOddsHit(p.prob_hit);
        setLogOddsMiss(p.prob_miss);
        setFreeThreshold(p.free_threshold);
        setOccupancyThreshold(p.occupancy_threshold);
        setClampingThresholds(p.min_clamp, p.max_clamp);
        setBlockSize(p.block_size);
        setLockAtMaxClamp(p.lock_at_max_clamp);
      }

      template <typename OTHER_CELL_VALUE>
      Grid2D(const Grid2D<OTHER_CELL_VALUE>& that)
      {
        width = that.width;
        height = that.height;
        origin = that.origin;
        resolution = that.resolution;

        data.resize(that.data.size(), CELL_VALUE{});

        setLogOddsHit(that.probability(that.log_odds_hit));
        setLogOddsMiss(that.probability(that.log_odds_miss));
        setFreeThreshold(that.probability(that.free_threshold));
        setOccupancyThreshold(that.probability(that.occupancy_threshold));
        setClampingThresholds(that.probability(that.min_clamp),
                              that.probability(that.max_clamp));
        setBlockSize(that.block_size);
        setLockAtMaxClamp(that.lock_at_max_clamp);

        for (int i = 0; i < data.size(); ++i)
        {
          data[i].logodds = that.data[i].logodds;
        }
      }

      inline Grid2D::Ptr createSubmap(const BoundingBox& b) const
      {
        auto bbx = getBBX().intersect(b);

        // If the bounding box does not intersect the grid, return an empty
        // grid.
        //
        // Else, we may run into unsigned overflows when calculating the grid
        // size.
        if (bbx.isEmpty(resolution))
        {
          Grid2D::Ptr out = Grid2D::Ptr(new Grid2D(0, 0, resolution, bbx.min));

          out->load_time = load_time;
          out->log_odds_hit = log_odds_hit;
          out->log_odds_miss = log_odds_miss;
          out->free_threshold = free_threshold;
          out->occupancy_threshold = occupancy_threshold;
          out->min_clamp = min_clamp;
          out->max_clamp = max_clamp;
          out->block_size = block_size;

          return out;
        }

        Cell cl = w2c(bbx.min);
        Cell cu = w2c(bbx.max);

        // Remove the buffer added by getBBX if on boundary
        if (cu.row == height && cu.row != 0)
          cu.row--;
        if (cu.col == width && cu.col != 0)
          cu.col--;

        unsigned int dr = cu.row - cl.row + 1;
        unsigned int dc = cu.col - cl.col + 1;

        Grid2D::Ptr s = Grid2D::Ptr(new Grid2D(dc, dr, resolution, c2w(cl)));

        s->load_time = load_time;
        s->log_odds_hit = log_odds_hit;
        s->log_odds_miss = log_odds_miss;
        s->free_threshold = free_threshold;
        s->occupancy_threshold = occupancy_threshold;
        s->min_clamp = min_clamp;
        s->max_clamp = max_clamp;
        s->block_size = block_size;

        unsigned int ndata = s->data.size();
        for (unsigned int i = 0; i < ndata; i++)
        {
          unsigned int r = i / dc;
          unsigned int c = i % dc;
          s->data[r * dc + c] = data[(cl.row + r) * width + (cl.col + c)];
        }

        return s;
      }

      inline float logodds(float probability) const
      {
        return logf(probability / (1.0f - probability));
      }

      inline float probability(float logodds) const
      {
        return 1.0f - (1.0f / (1.0f + expf(logodds)));
      }

      inline void setClampingThresholds(float min, float max)
      {
        min_clamp = logodds(min);
        max_clamp = logodds(max);
      }

      inline void setOccupancyThreshold(float p)
      {
        occupancy_threshold = logodds(p);
      }

      inline void setFreeThreshold(float p)
      {
        free_threshold = logodds(p);
      }

      inline void setLogOddsHit(float p)
      {
        log_odds_hit = logodds(p);
      }

      inline void setLogOddsMiss(float p)
      {
        log_odds_miss = logodds(p);
      }

      inline void setBlockSize(unsigned int bs)
      {
        block_size = bs;
      }

      inline void setLockAtMaxClamp(bool lock)
      {
        lock_at_max_clamp = lock;
      }

      inline bool inQ(const Cell& c) const
      {
        return ((c.col < width) && (c.row < height));
      }

      inline bool inQ(const Point& p) const
      {
        return getBBX().inQ(p);
      }

      inline bool inQ(const BoundingBox& b) const
      {
        return getBBX().inQ(b);
      }

      inline bool occupiedQ(unsigned int index) const
      {
        return data[index].logodds > occupancy_threshold;
      }

      inline bool occupiedQ(const Cell& c) const
      {
        return get(c).logodds > occupancy_threshold;
      }

      inline bool occupiedQ(const Point& p) const
      {
        return get(p).logodds > occupancy_threshold;
      }

      inline bool freeQ(unsigned int index) const
      {
        return data[index].logodds < free_threshold;
      }

      inline bool freeQ(const Cell& c) const
      {
        return get(c).logodds < free_threshold;
      }

      inline bool freeQ(const Point& p) const
      {
        return get(p).logodds < free_threshold;
      }

      inline bool unknownQ(unsigned int index) const
      {
        CELL_VALUE v = data[index];
        return (v.logodds < occupancy_threshold) &&
               (v.logodds > free_threshold);
      }

      inline bool unknownQ(const Cell& c) const
      {
        CELL_VALUE v = get(c);
        return (v.logodds < occupancy_threshold) &&
               (v.logodds > free_threshold);
      }

      inline bool unknownQ(const Point& p) const
      {
        CELL_VALUE v = get(p);
        return (v.logodds < occupancy_threshold) &&
               (v.logodds > free_threshold);
      }

      inline CELL_VALUE get(unsigned int index) const
      {
        return data[index];
      }

      inline CELL_VALUE get(const Cell& c) const
      {
        return data[c.row * width + c.col];
      }

      inline CELL_VALUE get(const Point& p) const
      {
        // return get(w2c(p));
        return data[static_cast<unsigned int>((p.y - origin.y) / resolution) *
                        width +
                    static_cast<unsigned int>((p.x - origin.x) / resolution)];
      }

      inline Cell getCell(unsigned int index) const
      {
        return Cell(index / width, index % width);
      }

      inline Point getPoint(unsigned int index) const
      {
        return c2w(getCell(index));
      }

      inline unsigned int getIndex(const Cell& c) const
      {
        return c.row * width + c.col;
      }

      inline unsigned int getIndex(const Point& p) const
      {
        return getIndex(w2c(p));
      }

      inline cell_state_t getCellState(unsigned int index) const
      {
        CELL_VALUE v = get(index);
        if (v.logodds > occupancy_threshold)
          return OCCUPIED;
        if (v.logodds < free_threshold)
          return FREE;
        return UNKNOWN;
      }

      inline cell_state_t getCellState(const Cell& c) const
      {
        CELL_VALUE v = get(c);
        if (v.logodds > occupancy_threshold)
          return OCCUPIED;
        if (v.logodds < free_threshold)
          return FREE;
        return UNKNOWN;
      }

      inline unsigned int getWidth() const
      {
        return width;
      }

      inline unsigned int getHeight() const
      {
        return height;
      }

      inline float getResolution() const
      {
        return resolution;
      }

      inline void set(unsigned int index, const CELL_VALUE& v)
      {
        data[index] = v;
      }

      inline void set(const Cell& c, const CELL_VALUE& v)
      {
        data[c.row * width + c.col] = v;
      }

      inline void set(const Point& p, const CELL_VALUE& v)
      {
        set(w2c(p), v);
      }

      void reset()
      {
        data.clear();
        width = height = 0;
        origin.x = origin.y = 0;
        load_time = 0.0;
      }

      void resize(const Cell& c)
      {
        width = c.col;
        height = c.row;
        data.resize(width * height, CELL_VALUE{});
      }

      inline bbx_t getBBX() const
      {
        if (data.size() == 0)
          return bbx_t(origin, origin);  // return a point
        // account for cell width
        Point buffer(resolution, resolution);
        return bbx_t(origin, c2w(getCell(data.size() - 1)) + buffer);
      }

      void extend(const bbx_t& newbb)
      {
        bbx_t bb = getBBX();
        bbx_t bb_old(bb);

        if (bb.min.x > newbb.min.x)
          bb.min.x -= ceilf((bb.min.x - newbb.min.x) / resolution) * resolution;
        if (bb.min.y > newbb.min.y)
          bb.min.y -= ceilf((bb.min.y - newbb.min.y) / resolution) * resolution;
        if (bb.max.x < newbb.max.x)
          bb.max.x += ceilf((newbb.max.x - bb.max.x) / resolution) * resolution;
        if (bb.max.y < newbb.max.y)
          bb.max.y += ceilf((newbb.max.y - bb.max.y) / resolution) * resolution;

        float resolution_inv = 1.0f / resolution;

        unsigned int new_width = width;
        unsigned int new_height = height;

        if (bb.min.x < bb_old.min.x)
        {
          unsigned int ext = static_cast<unsigned int>(
              roundf((bb_old.min.x - bb.min.x) * resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_width += ext;
          bb.min.x = bb_old.min.x - ext * resolution;
        }

        if (bb.max.x > bb_old.max.x)
        {
          unsigned int ext = static_cast<unsigned int>(
              roundf((bb.max.x - bb_old.max.x) * resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_width += ext;
        }

        if (bb.min.y < bb_old.min.y)
        {
          unsigned int ext = static_cast<unsigned int>(
              roundf((bb_old.min.y - bb.min.y) * resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_height += ext;
          bb.min.y = bb_old.min.y - ext * resolution;
        }

        if (bb.max.y > bb_old.max.y)
        {
          unsigned int ext = static_cast<unsigned int>(
              roundf((bb.max.y - bb_old.max.y) * resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_height += ext;
        }

        if ((new_width == width) && (new_height == height))
          return;

        unsigned int dx = static_cast<unsigned int>(
            roundf((bb_old.min.x - bb.min.x) * resolution_inv));
        unsigned int dy = static_cast<unsigned int>(
            roundf((bb_old.min.y - bb.min.y) * resolution_inv));

        Grid2D g(new_width, new_height, resolution, bb.min);

        unsigned int ndata = data.size();
        for (unsigned int i = 0; i < ndata; i++)
          g.set(Cell(i / width + dy, i % width + dx), data[i]);

        width = g.width;
        height = g.height;
        resolution = g.resolution;
        origin = g.origin;
        data.swap(g.data);
      }

      inline void updateHit(const Cell& c)
      {
        updateLogOdds(c, log_odds_hit);
      }

      inline void updateHit(const Point& p)
      {
        updateLogOdds(Cell((unsigned int)((p.y - origin.y) / resolution),
                           (unsigned int)((p.x - origin.x) / resolution)),
                      log_odds_hit);
      }

      inline void updateMiss(const Cell& c)
      {
        updateLogOdds(c, log_odds_miss);
      }

      inline void updateMiss(const Point& p)
      {
        updateLogOdds(Cell((unsigned int)((p.y - origin.y) / resolution),
                           (unsigned int)((p.x - origin.x) / resolution)),
                      log_odds_miss);
      }

      inline void updateLogOdds(const Cell& c, const float& update)
      {
        CELL_VALUE v = get(c);

        // In static environments with perfect(ish) sensing, we may want to lock
        // cells as occupied once we reach a certain confidence level because
        // observations from grazing rays could cause the cell to be marked
        // errantly as free.
        if (lock_at_max_clamp && v.logodds == max_clamp)
          return;

        // Limit value to min/max range
        v.logodds =
            std::max(min_clamp, std::min(max_clamp, v.logodds + update));
        set(c, v);
      }

      std::pair<bool, std::vector<Cell>> getRay(const Point& start,
                                                const Point& end)
      {
        std::vector<Cell> raycells;
        bool success = getRay(start, end, raycells);
        return std::make_pair(success, raycells);
      }

      bool getRay(const Point& start, const Point& end,
                  std::vector<Cell>& raycells) const
      {
        // Length
        float mag = hypotf(end.x - start.x, end.y - start.y);
        Point dir = (end - start) / mag;

        // From "A Fast Voxel Traversal Algorithm for Ray Tracing," Amanatides &
        // Woo

        // Start, end cell centers
        Cell c_start = w2c(start);
        Cell c_end = w2c(end);
        Cell c = c_start;

        // Don't check the end point, we will terminate once the ray leaves the
        // map Make sure the start point is in the map
        if (!inQ(c_start))
        {
          return false;
        }

        // compute number of cells to be added using larger datatype to prevent
        // overflow due to large rays
        unsigned long num_cells =
            1 + std::abs((long)c_end.row - (long)c_start.row) +
            std::abs((long)c_end.col - (long)c_start.col);
        num_cells = std::min(num_cells, (unsigned long)width + height);
        raycells.reserve(num_cells);

        // Handle horizontal and vertical rays
        bool search_row = c_start.row == c_end.row ? false : true;
        bool search_col = c_start.col == c_end.col ? false : true;

        // Ray is in only one cell
        if (!search_row && !search_col)
        {
          raycells.push_back(c_start);
          return true;
        }

        Point tmax, tdelta;

        Point cb = c2w(c);

        int step_col = -1;
        if (dir.x > 0.0f)
        {
          step_col = 1;
          cb.x += resolution;
        }

        int step_row = -1;
        if (dir.y > 0.0f)
        {
          step_row = 1;
          cb.y += resolution;
        }

        if (search_col)
        {
          tmax.x = (cb.x - start.x) * (1.0f / dir.x);
          tdelta.x = resolution * static_cast<float>(step_col) * (1.0f / dir.x);
        }

        if (search_row)
        {
          tmax.y = (cb.y - start.y) * (1.0f / dir.y);
          tdelta.y = resolution * static_cast<float>(step_row) * (1.0f / dir.y);
        }

        typedef enum
        {
          YY,
          YN,
          NY,
          NN
        } search_mode_t;
        search_mode_t mode;
        if (search_row)
          mode = search_col ? YY : YN;
        else
          mode = search_col ? NY : NN;

        while (true)
        {
          // Raycast until the edge of the map and escape before inserting the
          // cell if an invalid cell is reached
          if (!inQ(c))
          {
            return true;
          }

          // Add c to the cells in the raycells
          raycells.push_back(c);

          // escape after inserting the cell once reaching the end of the ray
          // (this makes the ray effectively a closed set)
          if ((c.col == c_end.col) && (c.row == c_end.row))
            break;

          if (raycells.size() >= num_cells)
          {
            break;
          }

          typedef enum
          {
            ROW,
            COL
          } update_t;
          update_t um;

          switch (mode)
          {
            case YN:
              um = ROW;
              break;
            case NY:
              um = COL;
              break;
            case YY:
            {
              if (tmax.x < tmax.y)
                um = COL;
              else
                um = ROW;
              break;
            }
            case NN:
            {
              fprintf(stderr, "[Grid2D] Impossible case - NN\n");
              return false;
            }
          }

          switch (um)
          {
            case ROW:
            {
              c.row += step_row;
              tmax.y += tdelta.y;
              break;
            }
            case COL:
            {
              c.col += step_col;
              tmax.x += tdelta.x;
              break;
            }
          }
        }

        return true;
      }

      void addRay(const Point& start, const Point& end, float max_range)
      {
        // Length
        float mag = hypotf(end.x - start.x, end.y - start.y);
        Point dir = (end - start) / mag;

        // Ray length exceeds max value, cut off
        bool overlength =
            ((mag > max_range) && (max_range > 0.0f)) ? true : false;

        Point ep(end);
        if (overlength)
        {
          // -1e-6 here to avoid exact cases that arise in simulation
          ep = start + (max_range - 1e-6) * dir;
        }

        std::vector<Cell> raycells;
        if (!getRay(start, ep, raycells))
          return;

        for (size_t i = 0; i < raycells.size() - 1; i++)
          updateMiss(raycells[i]);

        // Only register a hit if magnitude is less than max range
        if (!overlength)
          updateHit(raycells.back());
      }

      //////// WARNING ////////
      // this function assumes the size of the grid is fixed.
      // It does NOT extend the bounding box extents
      // Use addRegisteredPointCloud inside Grid3DROS if you need
      // this functionality
      void
      addRegisteredPointCloud(const Point& start_pt,
			      const std::vector<Point>& end_pts,
			      const float& max_range)
      {
	for (int i = 0; i < end_pts.size(); ++i)
	{
	  bool shortened_endpoint = false;
	  bool shortened_startpoint = false;
	  Point p0 = start_pt;
	  Point p1 = end_pts[i];
	  float raynorm;
	  if (intersect(p0, p1, shortened_endpoint, shortened_startpoint, raynorm)
	  )
	    addRay(p0, p1, max_range, shortened_endpoint, shortened_startpoint, raynorm);
	}
      }

      std::vector<Cell>
      addRay(const Point& start, const Point& end,
	     const float max_range, const bool& shortened_endpoint,
	     const bool& shortened_startpoint, const float& raynorm)
      {
	// Length
	Point err = end - start;
	float mag = sqrtf(err.x*err.x + err.y*err.y);
	Point dir = (end - start)/mag;

	// Ray length exceeds max value, cut off
	bool overlength = ((raynorm > max_range) && (max_range > 0.0f)) ? true : false;

        Point ep(end);
        if (overlength && (max_range < mag) )
        {
          // -1e-6 here to avoid exact cases that arise in simulation
          ep = start + (max_range - 1e-6)*dir;
        }

	std::vector<Cell> raycells;
	getRay(start, ep, raycells);
	if (raycells.size() == 0) return raycells;

	for (int32_t j = 0; j < ((int32_t)raycells.size())-1; ++j)
	  updateMiss(raycells[j]);

	if (!shortened_endpoint && !overlength)
	  updateHit(raycells.back());

	return raycells;
      }

      /////////////////////////////////////////////////////////////
      // Liang Barsky Algorithm for Line Clipping
      // Algorithm from https://en.wikipedia.org/wiki/Liang%E2%80%93Barsky_algorithm
      /////////////////////////////////////////////////////////////
      bool intersect(Point& pp1, Point& pp2, bool& shorten_endpoint,
		     bool& shorten_startpoint, float& raynorm)
      {
	using namespace std;

	bbx_t bbx = getBBX();

        // this function gives the maximum
        auto maxi = [](float arr[], int n)
	{
	  float m = 0;
	  for (int i = 0; i < n; ++i)
	    if (m < arr[i])
	      m = arr[i];
	  return m;
	};

        // this function gives the minimum
	auto mini = [](float arr[], int n)
	{
	  float m = 1;
	  for (int i = 0; i < n; ++i)
	    if (m > arr[i])
	      m = arr[i];
	  return m;
	};

        // checks if point is contained within bounding box
	auto outsideBBX = [](Point p, bbx_t bbx)
	{
	  if (p.x >= bbx.min.x && p.x <= bbx.max.x &&
	      p.y >= bbx.min.y && p.y <= bbx.max.y)
	    return false;
	  return true;
	};

	shorten_endpoint = false;
	shorten_startpoint = false;

	float xmin = bbx.min.x;
	float ymin = bbx.min.y;
	float xmax = bbx.max.x;
	float ymax = bbx.max.y;

	float x1 = pp1.x;
	float y1 = pp1.y;
	float x2 = pp2.x;
	float y2 = pp2.y;

	float p1 = -(x2 - x1);
	float p2 = -p1;
	float p3 = -(y2 - y1);
	float p4 = -p3;

	float q1 = x1 - xmin;
	float q2 = xmax - x1;
	float q3 = y1 - ymin;
	float q4 = ymax - y1;

	float posarr[5], negarr[5];
	int posind = 1, negind = 1;
	posarr[0] = 1;
	negarr[0] = 0;

	if ( (p1 == 0 && q1 < 0) || (p2 == 0 && q2 < 0) ||
	     (p3 == 0 && q3 < 0) || (p4 == 0 && q4 < 0))
	{
	  return false;
	}

	if (p1 != 0)
	{
	  float r1 = q1 / p1;
	  float r2 = q2 / p2;
	  if (p1 < 0) {
	    negarr[negind++] = r1; // for negative p1, add it to negative array
	    posarr[posind++] = r2; // and add p2 to positive array
	  }
	  else {
	    negarr[negind++] = r2;
	    posarr[posind++] = r1;
	  }
	}
	if (p3 != 0)
	{
	  float r3 = q3 / p3;
	  float r4 = q4 / p4;
	  if (p3 < 0)
	  {
	    negarr[negind++] = r3;
	    posarr[posind++] = r4;
	  }
	  else {
	    negarr[negind++] = r4;
	    posarr[posind++] = r3;
	  }
	}

	float xn1, yn1, xn2, yn2;
	float rn1, rn2;
	rn1 = maxi(negarr, negind); // maximum of negative array
	rn2 = mini(posarr, posind); // minimum of positive array

	if (rn1 > rn2)
	{ // reject
	  // Line is outside the clipping window
	  return false;
	}

	if (outsideBBX(pp1, bbx))
	  shorten_startpoint = true;

	if (outsideBBX(pp2, bbx))
	  shorten_endpoint = true;

	xn1 = x1 + p2 * rn1;
	yn1 = y1 + p4 * rn1; // computing new points

	xn2 = x1 + p2 * rn2;
	yn2 = y1 + p4 * rn2;

	pp1 = Point(xn1, yn1);
	pp2 = Point(xn2, yn2);

	raynorm = std::sqrt( (pp1.x - pp2.x)*(pp1.x - pp2.x) +
			     (pp1.y - pp2.y)*(pp1.y - pp2.y) );
	return true;
      }

      bool lineSearch(const Point& start, const Point& end,
                      const cell_state_t& s) const
      {
        std::vector<Cell> raycells;
        if (!getRay(start, end, raycells))
          return false;

        for (size_t i = 0; i < raycells.size(); i++)
          if (getCellState(raycells[i]) == s)
            return true;

        return false;
      }

      // World to Map
      inline Point w2m(const Point& in) const
      {
        return Point(in.x - origin.x, in.y - origin.y);
      }

      // Map to World
      inline Point m2w(const Point& in) const
      {
        return Point(in.x + origin.x, in.y + origin.y);
      }

      // Cell Center to Map
      inline Point c2m(const Cell& in) const
      {
        return Point(in.col * resolution, in.row * resolution);
      }

      // Map to Cell
      inline Cell m2c(const Point& in) const
      {
        // Truncate and round down
        return Cell((unsigned int)(in.y / resolution),
                    (unsigned int)(in.x / resolution));
      }

      // World to Cell
      inline Cell w2c(const Point& in) const
      {
        // return m2c(w2m(in));
        return Cell((unsigned int)((in.y - origin.y) / resolution),
                    (unsigned int)((in.x - origin.x) / resolution));
      }

      // Cell to World
      inline Point c2w(const Cell& in) const
      {
        // return m2w(c2m(in));
        return Point(fmaf(static_cast<float>(in.col), resolution, origin.x),
                     fmaf(static_cast<float>(in.row), resolution, origin.y));
      }

      // index to world
      inline Point i2w(unsigned int index) const
      {
        return c2w(getCell(index));
      }

      // world to index
      inline unsigned int w2i(const Point& in) const
      {
        return getIndex(w2c(in));
      }
    };
  }  // namespace grid2d
}  // namespace map_utils
