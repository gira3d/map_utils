#ifndef MAP_UTILS_GRID_3D_H
#define MAP_UTILS_GRID_3D_H

#include <boost/version.hpp>
#include <cmath>
#include <stdio.h>

#include <vector>
#include <boost/unordered_set.hpp>
#include "Grid3DCellValue.h"
#include "Grid3DCell.h"
#include "Grid3DPoint.h"
#include "Grid3DBoundingBox.h"
#include "Grid3DParameters.h"

namespace map_utils
{
  namespace grid3d
  {

    template <typename CELL_VALUE = CellValue>
    struct Grid3D
    {
      typedef boost::shared_ptr<Grid3D> Ptr;
      typedef boost::shared_ptr<const Grid3D> ConstPtr;

      typedef enum {FREE, UNKNOWN, OCCUPIED} cell_state_t;

      std::vector<CELL_VALUE> data;
      unsigned int width, height, depth;
      float resolution;
      double load_time = 0.0;
      Point origin;
      float log_odds_hit, log_odds_miss;
      float free_threshold, occupancy_threshold;
      float min_clamp, max_clamp;
      unsigned int block_size;
      bool track_changes;
      typedef boost::unordered_set<unsigned int> changeset_t;
      changeset_t changes;
      bool lock_at_max_clamp;

      Grid3D() : width(0), height(0), depth(0), resolution(0.0f),
        track_changes(false), lock_at_max_clamp(false) { }

      Grid3D(params_t p)
      {
	width = p.width;
	height = p.height;
	depth = p.depth;
	origin = p.origin;
	resolution = p.resolution;

	data.resize(width*height*depth, CELL_VALUE{});

	setLogOddsHit(p.prob_hit);
	setLogOddsMiss(p.prob_miss);
	setFreeThreshold(p.free_threshold);
	setOccupancyThreshold(p.occupancy_threshold);
	setClampingThresholds(p.min_clamp, p.max_clamp);
	setBlockSize(p.block_size);
	setTrackChanges(p.track_changes);
	setLockAtMaxClamp(p.lock_at_max_clamp);

	if (track_changes)
	  changes.reserve(width*height*depth);
      }

      Grid3D(unsigned int w, unsigned int h, unsigned int d) :
        width(w), height(h), depth(d), resolution(0.0f), block_size(1),
        track_changes(false), lock_at_max_clamp(false)
      {
        data.resize(width*height*depth, CELL_VALUE{});
#if BOOST_VERSION >= 105000
        changes.reserve(width*height*depth);
#endif
      }
      Grid3D(unsigned int w, unsigned int h, unsigned int d, float r) :
        width(w), height(h), depth(d), resolution(r), block_size(1),
        track_changes(false), lock_at_max_clamp(false)
      {
        data.resize(width*height*depth, CELL_VALUE{});
#if BOOST_VERSION >= 105000
        changes.reserve(width*height*depth);
#endif
      }
      Grid3D(unsigned int w, unsigned int h, unsigned int d, float r, const Point& p) :
        width(w), height(h), depth(d), resolution(r), origin(p), block_size(1),
        track_changes(false), lock_at_max_clamp(false)
      {
        data.resize(width*height*depth, CELL_VALUE{});
#if BOOST_VERSION >= 105000
        changes.reserve(width*height*depth);
#endif
      }

      template<typename OTHER_CELL_VALUE>
      Grid3D(const Grid3D<OTHER_CELL_VALUE>& that)
      {
	width = that.width;
	height = that.height;
	depth = that.depth;
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
	setTrackChanges(that.track_changes);
	setLockAtMaxClamp(that.lock_at_max_clamp);

	if (track_changes)
	  changes.reserve(width*height*depth);

	for (int i = 0; i < data.size(); ++i)
	{
	  data[i].logodds = that.data[i].logodds;
	}
      }

      inline Grid3D::Ptr createSubmap(const BoundingBox& b) const
      {
        auto bbx = getBBX().intersect(b);

        // If the bounding box does not intersect the grid, return an empty
        // grid.
        //
        // Else, we may run into unsigned overflows when calculating the grid
        // size.
        //
        // Note that this is only triggered if the intersection is the empty set
        // (versus having zero volume)
        if (!bbx.isNonEmpty())
        {
          Grid3D::Ptr out =
            Grid3D::Ptr(new Grid3D(0, 0, 0, resolution, bbx.min));

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
        if (cu.row == height) cu.row--;
        if (cu.col == width) cu.col--;
        if (cu.slice == depth) cu.slice--;

        unsigned int dr = cu.row - cl.row + 1;
        unsigned int dc = cu.col - cl.col + 1;
        unsigned int ds = cu.slice - cl.slice + 1;

        Grid3D::Ptr out =
          Grid3D::Ptr(new Grid3D(dc, dr, ds, resolution, c2w(cl)));

        out->load_time = load_time;
        out->log_odds_hit = log_odds_hit;
        out->log_odds_miss = log_odds_miss;
        out->free_threshold = free_threshold;
        out->occupancy_threshold = occupancy_threshold;
        out->min_clamp = min_clamp;
        out->max_clamp = max_clamp;
        out->block_size = block_size;

        unsigned int this_row_stride = this->width * this->depth;
        unsigned int this_col_stride = this->depth;

        // To minimize computation we compute the index outside of the inner
        // loop
        // To do so we jump through remaining slices in a column and columns in
        // a row
        unsigned int this_row_delta = this_row_stride - this_col_stride * dc;
        unsigned int this_col_delta = this_col_stride - ds;

        unsigned int out_index = 0;
        unsigned int this_index = this->getIndex(cl);
        for (unsigned int r = 0; r < dr; r++){
          for (unsigned int c = 0; c < dc; c++){
            for (unsigned int s = 0; s < ds; s++){
              out->data[out_index] = this->data[this_index];
              out_index += 1;
              this_index += 1;
            }
            this_index += this_col_delta;
          }
          this_index += this_row_delta;
        }

        return out;
      }

      inline float logodds(float probability) const
      {
        return logf(probability/(1.0f - probability));
      }

      inline float probability(float logodds) const
      {
        return 1.0f/(1.0f + expf(-logodds));
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

      inline void setTrackChanges(bool track)
      {
        track_changes = track;
      }

      inline void setLockAtMaxClamp(bool lock)
      {
        lock_at_max_clamp = lock;
      }

      inline bool inQ(const Cell& c) const
      {
        return ((c.col < width) && (c.row < height) && (c.slice < depth));
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
        return (v.logodds < occupancy_threshold) && (v.logodds > free_threshold);
      }

      inline bool unknownQ(const Cell& c) const
      {
        CELL_VALUE v = get(c);
        return (v.logodds < occupancy_threshold) && (v.logodds > free_threshold);
      }

      inline bool unknownQ(const Point& p) const
      {
        CELL_VALUE v = get(p);
        return (v.logodds < occupancy_threshold) && (v.logodds > free_threshold);
      }

      inline CELL_VALUE get(unsigned int index) const
      {
        return data[index];
      }

      inline CELL_VALUE get(const Cell& c) const
      {
        return data[(c.row*width + c.col)*depth + c.slice];
      }

      inline CELL_VALUE get(const Point& p) const
      {
        return get(Cell((unsigned int)((p.y - origin.y)/resolution),
                        (unsigned int)((p.x - origin.x)/resolution),
                        (unsigned int)((p.z - origin.z)/resolution)));
      }

      inline Cell getCell(unsigned int index) const
      {
        return Cell(index / (width*depth), (index / depth) % width, index % depth);
      }

      inline Point getPoint(unsigned int index) const
      {
        return c2w(getCell(index));
      }

      inline unsigned int getIndex(const Cell& c) const
      {
        return (c.row*width + c.col)*depth + c.slice;
      }

      inline unsigned int getIndex(const Point& p) const
      {
        return getIndex(Cell((unsigned int)((p.y - origin.y)/resolution),
                             (unsigned int)((p.x - origin.x)/resolution),
                             (unsigned int)((p.z - origin.z)/resolution)));
      }

      inline cell_state_t getCellState(unsigned int index) const
      {
        CELL_VALUE v = get(index);
        if (v.logodds > occupancy_threshold) return OCCUPIED;
        if (v.logodds < free_threshold) return FREE;
        return UNKNOWN;
      }

      inline cell_state_t getCellState(const Cell& c) const
      {
        CELL_VALUE v = get(c);
        if (v.logodds > occupancy_threshold) return OCCUPIED;
        if (v.logodds < free_threshold) return FREE;
        return UNKNOWN;
      }

      // Added to integrate the incremental grid3dmsg assuming
      // the dimensions of grids across robots are exactly the same size
      // and have exactly the same origins.
      inline void set(const unsigned int index, const CELL_VALUE& v)
      {
	if (index < data.size())
	{
	  // Only insert change into changeset when value changes
	  // Note that this check must occur before we update the value!
	  if (track_changes && data[index].logodds != v.logodds) changes.insert(index);
	  data[index] = v;
	}
      }

      inline void set(const Cell& c, const float& l)
      {
        unsigned int index = (c.row*width + c.col)*depth + c.slice;

        // Only insert change into changeset when value changes
        // Note that this check must occur before we update the value!
        if (track_changes && data[index].logodds != l) changes.insert(index);

        data[index].logodds = l;
      }

      inline void set(const Cell& c, const CELL_VALUE& v)
      {
        unsigned int index = (c.row*width + c.col)*depth + c.slice;

        // Only insert change into changeset when value changes
        // Note that this check must occur before we update the value!
        if (track_changes && data[index].logodds != v.logodds) changes.insert(index);

        data[index] = v;
      }

      inline void set(const Point& p, const CELL_VALUE& v)
      {
        set(Cell((unsigned int)((p.y - origin.y)/resolution),
                 (unsigned int)((p.x - origin.x)/resolution),
                 (unsigned int)((p.z - origin.z)/resolution)), v);
      }

      void resetChangeSet()
      {
        changes.clear();
      }

      void reset()
      {
        data.clear();
        changes.clear();
        width = height = depth = 0;
        origin.x = origin.y = origin.z = 0.0f;
        load_time = 0.0;
      }

      void resize(unsigned int w, unsigned int h, unsigned int d)
      {
        width = w;
        height = h;
        depth = d;
        data.resize(width*height*depth, CELL_VALUE{});
      }

      inline bbx_t getBBX() const
      {
        if (data.size() == 0)
          return bbx_t(origin, origin); // return a point
        // account for cell width
        Point buffer(resolution, resolution, resolution);
        return bbx_t(origin, c2w(getCell(data.size()-1)) + buffer);
      }

      void extend(const bbx_t& newbb)
      {
        bbx_t bb = getBBX();
        bbx_t bb_old(bb);

        if (bb.min.x > newbb.min.x)
          bb.min.x -= ceilf((bb.min.x - newbb.min.x)/resolution)*resolution;
        if (bb.min.y > newbb.min.y)
          bb.min.y -= ceilf((bb.min.y - newbb.min.y)/resolution)*resolution;
        if (bb.min.z > newbb.min.z)
          bb.min.z -= ceilf((bb.min.z - newbb.min.z)/resolution)*resolution;
        if (bb.max.x < newbb.max.x)
          bb.max.x += ceilf((newbb.max.x - bb.max.x)/resolution)*resolution;
        if (bb.max.y < newbb.max.y)
          bb.max.y += ceilf((newbb.max.y - bb.max.y)/resolution)*resolution;
        if (bb.max.z < newbb.max.z)
          bb.max.z += ceilf((newbb.max.z - bb.max.z)/resolution)*resolution;

        float resolution_inv = 1.0f/resolution;

        unsigned int new_width = width;
        unsigned int new_height = height;
        unsigned int new_depth = depth;

        if (bb.min.x < bb_old.min.x)
        {
          unsigned int ext =
            static_cast<unsigned int>(roundf((bb_old.min.x - bb.min.x)*resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_width += ext;
          bb.min.x = bb_old.min.x - ext*resolution;
        }

        if (bb.max.x > bb_old.max.x)
        {
          unsigned int ext =
            static_cast<unsigned int>(roundf((bb.max.x - bb_old.max.x)*resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_width += ext;
        }

        if (bb.min.y < bb_old.min.y)
        {
          unsigned int ext =
            static_cast<unsigned int>(roundf((bb_old.min.y - bb.min.y)*resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_height += ext;
          bb.min.y = bb_old.min.y - ext*resolution;
        }

        if (bb.max.y > bb_old.max.y)
        {
          unsigned int ext =
            static_cast<unsigned int>(roundf((bb.max.y - bb_old.max.y)*resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_height += ext;
        }

        if (bb.min.z < bb_old.min.z)
        {
          unsigned int ext =
            static_cast<unsigned int>(roundf((bb_old.min.z - bb.min.z)*resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_depth += ext;
          bb.min.z = bb_old.min.z - ext*resolution;
        }

        if (bb.max.z > bb_old.max.z)
        {
          unsigned int ext =
            static_cast<unsigned int>(roundf((bb.max.z - bb_old.max.z)*resolution_inv));
          if (ext % block_size)
            ext += block_size - (ext % block_size);
          new_depth += ext;
        }

        if ((new_width == width) && (new_height == height) && (new_depth == depth))
          return;

        unsigned int dx =
          static_cast<unsigned int>(roundf((bb_old.min.x - bb.min.x)*resolution_inv));
        unsigned int dy =
          static_cast<unsigned int>(roundf((bb_old.min.y - bb.min.y)*resolution_inv));
        unsigned int dz =
          static_cast<unsigned int>(roundf((bb_old.min.z - bb.min.z)*resolution_inv));

        Grid3D g(new_width, new_height, new_depth, resolution, bb.min);

        unsigned int ndata = data.size();
        for (unsigned int i = 0; i < ndata; i++)
          g.set(Cell((i / (width*depth)) + dy,
                     ((i / depth) % width) + dx, i % depth + dz), data[i]);

        width = g.width;
        height = g.height;
        depth = g.depth;
        resolution = g.resolution;
        origin = g.origin;
        data.swap(g.data);
      }

      // resize is similar to extend but admits resizing to a smaller bounding
      // box and guarantees exact sizing rather than snapping to blocks
      void resize(const bbx_t& input_bb)
      {
        bbx_t new_bb = getBBX();
        bbx_t old_bb(new_bb);

        // compute nominal change to the boundaries of the bounding box
        Point delta_min = input_bb.min - old_bb.min;
        Point delta_max = input_bb.max - old_bb.max;

        auto snap_up = [this](float value)
        {
          return ceilf(value/resolution) * resolution;
        };
        auto snap_down = [this](float value)
        {
          return floorf(value/resolution) * resolution;
        };

        // snap boundaries of new bounding to grid resolution

        delta_max.x = snap_up(delta_max.x);
        delta_max.y = snap_up(delta_max.y);
        delta_max.z = snap_up(delta_max.z);

        delta_min.x = snap_down(delta_min.x);
        delta_min.y = snap_down(delta_min.y);
        delta_min.z = snap_down(delta_min.z);

        new_bb.min = old_bb.min + delta_min;
        new_bb.max = old_bb.max + delta_max;

        // snap the max up in case input was not a proper bounding box
        // (i.e. negative volume)
        new_bb.max.x = std::max(new_bb.min.x, new_bb.max.x);
        new_bb.max.y = std::max(new_bb.min.y, new_bb.max.y);
        new_bb.max.z = std::max(new_bb.min.z, new_bb.max.z);

        // return if the bounding box has not changed
        if (new_bb == old_bb)
          return;

        //////////////////////////////
        // Start building the new grid
        //////////////////////////////

        // compute the height, width, and depth as integers
        auto dist_to_cells = [this](float value)
        {
          return static_cast<int>(roundf(value/resolution));
        };

        Point new_size = new_bb.max - new_bb.min;
        unsigned int new_width  = dist_to_cells(new_size.x);
        unsigned int new_height = dist_to_cells(new_size.y);
        unsigned int new_depth  = dist_to_cells(new_size.z);

        Grid3D new_grid(new_width, new_height, new_depth, resolution, new_bb.min);

        // compute cell conversion
        auto new_to_old = old_bb.min - new_bb.min;

        int row_offset   = dist_to_cells(new_to_old.y);
        int col_offset   = dist_to_cells(new_to_old.x);
        int slice_offset = dist_to_cells(new_to_old.z);

        // compute bounds of the current grid
        Cell lower = w2c(old_bb.min);
        Cell upper = w2c(old_bb.max);

        // The upper corner of the bounding box is (up to rounding error) in the
        // lower corner of a cell outside the map
        if (upper.row   == height) upper.row--;
        if (upper.col   == width)  upper.col--;
        if (upper.slice == depth)  upper.slice--;

        // Loop over the old grid and copy values that are in the new grid
        // This is a little easier than intersecting the bounding boxes
        for(size_t ii = lower.row; ii <= upper.row; ++ii)
        {
          for(size_t jj = lower.col; jj <= upper.col; ++jj)
          {
            for(size_t kk = lower.slice; kk <= upper.slice; ++kk)
            {
              Cell old_cell(ii, jj, kk);
              // add vector in old to old origin in new
              Cell new_cell((int)ii + row_offset,
                            (int)jj + col_offset,
                            (int)kk + slice_offset);

              if(new_grid.inQ(new_cell))
              {
                new_grid.set(new_cell, get(old_cell));
              }
            }
          }
        }

        // We've neglected to set some values in the new grid and so do not swap
        // everything
        width      = new_grid.width;
        height     = new_grid.height;
        depth      = new_grid.depth;
        resolution = new_grid.resolution;
        origin     = new_grid.origin;
        data.swap(new_grid.data);
      }

      inline void updateHit(const Cell& c)
      {
        updateLogOdds(c, log_odds_hit);
      }

      inline void updateHit(const Point& p)
      {
        updateLogOdds(Cell((unsigned int)((p.y - origin.y)/resolution),
                           (unsigned int)((p.x - origin.x)/resolution),
                           (unsigned int)((p.z - origin.z)/resolution)), log_odds_hit);
      }

      inline void updateMiss(const Cell& c)
      {
        updateLogOdds(c, log_odds_miss);
      }

      inline void updateMiss(const Point& p)
      {
        updateLogOdds(Cell((unsigned int)((p.y - origin.y)/resolution),
                           (unsigned int)((p.x - origin.x)/resolution),
                           (unsigned int)((p.z - origin.z)/resolution)), log_odds_miss);
      }

      inline void updateLogOdds(const Cell& c, const float& update)
      {
        CELL_VALUE v = get(c);

        // In static environments with perfect(ish) sensing, we may want to lock
        // cells as occupied once we reach a certain confidence level because
        // observations from grazing rays could cause the cell to be marked
        // errantly as free.
        if(lock_at_max_clamp && v.logodds == max_clamp)
          return;

        // Limit value to min/max range
	v.logodds = std::max(min_clamp, std::min(max_clamp, v.logodds + update));
        set(c, v);
      }

      std::pair<bool, std::vector<Cell>> getRay(const Point& start,
                                                const Point& end)
      {
        std::vector<Cell> raycells;
        bool success = getRay(start, end, raycells);
        return std::make_pair(success, raycells);
      }

      bool getRay(const Point& start, const Point& end, std::vector<Cell>& raycells) const
      {
        // Length
        Point err = end - start;
        float mag = sqrtf(err.x*err.x + err.y*err.y + err.z*err.z);
        Point dir = (end - start)/mag;

        // From "A Fast Voxel Traversal Algorithm for Ray Tracing," Amanatides & Woo

        // Start, end cell centers
        Cell c_start = w2c(start);
        Cell c_end = w2c(end);
        Cell c = c_start;

        // Don't check the end point, we will terminate once the ray leaves the map
        // Make sure the start point is in the map
        if (!inQ(c_start))
        {
          return false;
        }

        // compute number of cells to be added using larger datatype to prevent
        // overflow due to large rays
        unsigned long num_cells = 1 + std::abs((long) c_end.row - (long) c_start.row)
                                    + std::abs((long) c_end.col - (long) c_start.col)
                                    + std::abs((long) c_end.slice - (long) c_start.slice);

        num_cells = std::min(num_cells, (unsigned long) width + height + depth);
        raycells.reserve(num_cells);

        // Handle horizontal and vertical rays
        bool search_row = c_start.row == c_end.row ? false : true;
        bool search_col = c_start.col == c_end.col ? false : true;
        bool search_slice = c_start.slice == c_end.slice ? false : true;

        // Ray is in only one cell
        if (!search_row && !search_col && !search_slice)
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

        int step_slice = -1;
        if (dir.z > 0.0f)
        {
          step_slice = 1;
          cb.z += resolution;
        }

        if (search_col)
        {
          tmax.x = (cb.x - start.x)*(1.0f/dir.x);
          tdelta.x = resolution*static_cast<float>(step_col)*(1.0f/dir.x);
        }

        if (search_row)
        {
          tmax.y = (cb.y - start.y)*(1.0f/dir.y);
          tdelta.y = resolution*static_cast<float>(step_row)*(1.0f/dir.y);
        }

        if (search_slice)
        {
          tmax.z = (cb.z - start.z)*(1.0f/dir.z);
          tdelta.z = resolution*static_cast<float>(step_slice)*(1.0f/dir.z);
        }

        typedef enum {YYY,YYN,YNY,YNN,NYY,NYN,NNY,NNN} search_mode_t;
        search_mode_t mode;
        if (search_row)
        {
          if (search_col)
            mode = search_slice ? YYY : YYN;
          else
            mode = search_slice ? YNY : YNN;
        }
        else
        {
          if (search_col)
            mode = search_slice ? NYY : NYN;
          else
            mode = search_slice ? NNY : NNN;
        }

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
          if ((c.col == c_end.col) &&
              (c.row == c_end.row) &&
              (c.slice == c_end.slice))
            break;

          if (raycells.size() >= num_cells)
          {
            break;
          }

          typedef enum {ROW,COL,SLI} update_t;
          update_t um;

          switch (mode)
          {
            case YNN: um = ROW; break;
            case NYN: um = COL; break;
            case NNY: um = SLI; break;
            case YYN:
            {
              if (tmax.x < tmax.y)
                um = COL;
              else
                um = ROW;
              break;
            }
            case YNY:
            {
              if (tmax.y < tmax.z)
                um = ROW;
              else
                um = SLI;
              break;
            }
            case NYY:
            {
              if (tmax.x < tmax.z)
                um = COL;
              else
                um = SLI;
              break;
            }
            case YYY:
            {
              if (tmax.x < tmax.y)
              {
                if (tmax.x < tmax.z)
                  um = COL;
                else
                  um = SLI;
              }
              else
              {
                if (tmax.y < tmax.z)
                  um = ROW;
                else
                  um = SLI;
              }
              break;
            }
            case NNN:
            {
              fprintf(stderr, "[Grid3D] Impossible case - NNN\n");
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
            case SLI:
              {
                c.slice += step_slice;
                tmax.z += tdelta.z;
                break;
              }
          }
        }

        return true;
      }

      void addRay(const Point& start, const Point& end, float max_range)
      {
        // Length
        Point err = end - start;
        float mag = sqrtf(err.x*err.x + err.y*err.y + err.z*err.z);
        Point dir = (end - start)/mag;

        // Ray length exceeds max value, cut off
        bool overlength = ((mag > max_range) && (max_range > 0.0f)) ? true : false;

        Point ep(end);
        if (overlength)
        {
          // -1e-6 here to avoid exact cases that arise in simulation
          ep = start + (max_range - 1e-6)*dir;
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
	float mag = sqrtf(err.x*err.x + err.y*err.y + err.z*err.z);
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

      // Checks that the line segment specified by the start point (p0) and
      // end point (p1) intersects the bounding box. If yes, the
      // function returns true. If no, the function returns false.
      //
      // Inputs: start point p0, end point p1, and shorten boolean
      // which tells us if the endpoint of the ray had to be shortened
      // to fit within the bounding box extents. This is used to
      // assign occupied or free values
      bool intersect(Point& p0, Point& p1, bool& shorten_endpoint, bool& shorten_startpoint, float& raynorm)
      {
	shorten_endpoint = false;
	shorten_startpoint = false;

	ray_t r = ray_t(p0, p1);
	Point d = p1-p0;
	raynorm = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
	float t1 = raynorm;

	float t0 = 0; float tmin = 0;
	float tmax = 0;

	bbx_t bbx = getBBX();
	if (!bbx.rayIntersect(r, t0, t1, tmin, tmax)) return false;

	// endpoint exceeds the bbx bounds
	if (tmax <= t1)
	{
	  shorten_endpoint = true;

	  if (inQ(w2c(p0 + tmax*r.direction)))
	  {
	    p1 = p0 + tmax*r.direction;
	  }
	  else
	  {
	    // shorten endpoint a bit to avoid issues with precision
	    // at the boundaries
	    p1 = p0 + (tmax-resolution/100)*r.direction;
	  }
	}

	// origin is outside of the bounding box
	if (tmin >= t0)
	{
	  shorten_startpoint = true;

	  if (inQ(w2c(p0 + tmin*r.direction)))
	    p0 += tmin*r.direction;
	  else
	    p0 += (tmin+resolution/2)*r.direction;
	}

	if (!inQ(w2c(p0)) || !inQ(w2c(p1)))
	{
	  return false;
	}

	return true;
      }

      bool lineSearch(const Point& start, const Point& end, const cell_state_t& s) const
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
        return Point(in.x - origin.x, in.y - origin.y, in.z - origin.z);
      }

      // Map to World
      inline Point m2w(const Point& in) const
      {
        return Point(in.x + origin.x, in.y + origin.y, in.z + origin.z);
      }

      // Cell Center to Map
      inline Point c2m(const Cell& in) const
      {
        return Point(in.col*resolution, in.row*resolution, in.slice*resolution);
      }

      // Map to Cell
      inline Cell m2c(const Point& in) const
      {
        // Truncate and round down
        return Cell((unsigned int)(in.y/resolution),
                    (unsigned int)(in.x/resolution),
                    (unsigned int)(in.z/resolution));
      }

      // World to Cell
      inline Cell w2c(const Point& in) const
      {
        //return m2c(w2m(in));
        return Cell((unsigned int)((in.y - origin.y)/resolution),
                    (unsigned int)((in.x - origin.x)/resolution),
                    (unsigned int)((in.z - origin.z)/resolution));
      }

      // Cell to World
      inline Point c2w(const Cell& in) const
      {
        //return m2w(c2m(in));
        return Point(fmaf(static_cast<float>(in.col), resolution, origin.x),
                     fmaf(static_cast<float>(in.row), resolution, origin.y),
                     fmaf(static_cast<float>(in.slice), resolution, origin.z));
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
  }
}
#endif
