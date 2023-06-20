#ifndef MAP_UTILS_GRID_2D_ROS_H
#define MAP_UTILS_GRID_2D_ROS_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_utils/GeometryUtils.h>

#include "Grid2D.h"

namespace map_utils
{
namespace grid2d
{
namespace ros
{
template <typename T = CellValue>
inline nav_msgs::OccupancyGrid::Ptr toOccupancyGrid(const Grid2D<T>& grid,
                                                    const std::string& frame_id)
{
  nav_msgs::OccupancyGrid::Ptr msg =
      nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid());

  msg->header.stamp = ::ros::Time(grid.load_time);
  msg->header.frame_id = frame_id;
  msg->info.map_load_time = ::ros::Time(grid.load_time);
  msg->info.resolution = grid.resolution;
  msg->info.origin.position.x = grid.origin.x;
  msg->info.origin.position.y = grid.origin.y;
  msg->info.width = grid.width;
  msg->info.height = grid.height;

  if ((msg->info.width == 0) || (msg->info.height == 0))
  {
    msg->info.width = 0;
    msg->info.height = 0;
    msg->data.clear();
    return msg;
  }

  msg->data.reserve(grid.data.size());
  for (unsigned int i = 0; i < grid.data.size(); i++)
  {
    switch (grid.getCellState(i))
    {
      case Grid2D<T>::FREE:
        msg->data.push_back(0);
        break;
      case Grid2D<T>::UNKNOWN:
        msg->data.push_back(-1);
        break;
      case Grid2D<T>::OCCUPIED:
        msg->data.push_back(100);
        break;
    }
  }

  return msg;
}

template <typename T = CellValue>
inline typename map_utils::grid2d::Grid2D<T>::Ptr
fromROS(const nav_msgs::OccupancyGrid& og, const float prob_hit,
        const float prob_miss, const float free_threshold,
        const float occupancy_threshold, const float min_clamp,
        const float max_clamp, const unsigned int block_size)
{
  typename map_utils::grid2d::Grid2D<T>::Ptr msg =
      map_utils::grid2d::Grid2D<T>::Ptr(new map_utils::grid2d::Grid2D<T>());

  msg->origin.x = og.info.origin.position.x;
  msg->origin.y = og.info.origin.position.y;
  msg->width = og.info.width;
  msg->height = og.info.height;
  msg->resolution = og.info.resolution;
  msg->load_time = og.info.map_load_time.toSec();

  msg->setLogOddsHit(prob_hit);
  msg->setLogOddsMiss(prob_miss);
  msg->setFreeThreshold(free_threshold);
  msg->setOccupancyThreshold(occupancy_threshold);
  msg->setClampingThresholds(min_clamp, max_clamp);
  msg->setBlockSize(block_size);

  if (msg->width == 0 || msg->height == 0)
  {
    msg->data.clear();
    return msg;
  }

  msg->data.reserve(og.data.size());
  for (unsigned int i = 0; i < og.data.size(); ++i)
  {
    map_utils::grid2d::CellValue v;
    if (og.data[i] == 0)  // Free
      v.logodds = msg->logodds(free_threshold - 1e-5f);
    else if (og.data[i] == 100)  // Occupied
      v.logodds = msg->logodds(occupancy_threshold + 1e-5f);
    else  // Unknown
      v.logodds = msg->logodds(0.5f);
    msg->data.push_back(v);
  }
  return msg;
}

template <typename T = CellValue>
inline sensor_msgs::PointCloud::Ptr toPointCloud(const Grid2D<T>& grid,
                                                 const std::string& frame_id)
{
  sensor_msgs::PointCloud::Ptr msg =
      sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud());

  msg->header.stamp = ::ros::Time(grid.load_time);
  msg->header.frame_id = frame_id;
  msg->points.reserve(grid.data.size());
  msg->channels.resize(1);
  msg->channels[0].name = "logodds";
  msg->channels[0].values.reserve(grid.data.size());

  float g2 = 0.5 * grid.resolution;

  for (unsigned int r = 0; r < grid.height; r++)
    for (unsigned int c = 0; c < grid.width; c++)
    {
      Cell pc(r, c);
      Point pw = grid.c2w(pc);

      geometry_msgs::Point32 p;
      // Compensate for middle of point vs lower-left corner
      p.x = pw.x + g2;
      p.y = pw.y + g2;

      msg->points.push_back(p);
      msg->channels[0].values.push_back(grid.get(pc).logodds);
    }

  return msg;
}

struct RegisteredScan
{
  typedef boost::shared_ptr<RegisteredScan> Ptr;
  typedef boost::shared_ptr<const RegisteredScan> ConstPtr;

  geometry_utils::Transform2 sensor_origin;
  sensor_msgs::PointCloud cloud;
  float max_range;
};

template <typename T = CellValue>
inline void addRegisteredScan(const RegisteredScan::ConstPtr& scan,
                              Grid2D<T>& grid)
{
  bool use_range = false;
  if ((scan->cloud.channels.size() > 0) &&
      (scan->cloud.channels[0].name.compare("range") == 0))
    use_range = true;

  // Get the scan min and max points
  bbx_t bb = grid.getBBX();
  bbx_t bb_old = bb;

  Point start(scan->sensor_origin.translation(0),
              scan->sensor_origin.translation(1));

  bb.min.x = fminf(start.x, bb.min.x);
  bb.min.y = fminf(start.y, bb.min.y);
  bb.max.x = fmaxf(start.x, bb.max.x);
  bb.max.y = fmaxf(start.y, bb.max.y);

  unsigned int npoints = scan->cloud.points.size();
  for (unsigned int i = 0; i < npoints; i++)
  {
    Point end(scan->cloud.points[i].x, scan->cloud.points[i].y);

    if (use_range && (scan->cloud.channels[0].values[i] > scan->max_range))
    {
      Point err = end - start;
      end = start + scan->max_range * (err / hypotf(err.x, err.y));
    }

    bb.min.x = fminf(end.x, bb.min.x);
    bb.min.y = fminf(end.y, bb.min.y);
    bb.max.x = fmaxf(end.x, bb.max.x);
    bb.max.y = fmaxf(end.y, bb.max.y);
  }

  if (bb != bb_old)
    grid.extend(bb);

  for (unsigned int i = 0; i < npoints; i++)
    grid.addRay(start, Point(scan->cloud.points[i].x, scan->cloud.points[i].y),
                scan->max_range);

  grid.load_time = scan->cloud.header.stamp.toSec();
}

template <typename T = CellValue>
inline void addRegisteredScans(
    const std::vector<RegisteredScan::ConstPtr>& scans, Grid2D<T>& grid)
{
  unsigned int nscans = scans.size();
  if (nscans == 0)
    return;

  // Get the scan min and max points
  bbx_t bb = grid.getBBX();
  bbx_t bb_old(bb);

  for (unsigned int i = 0; i < nscans; i++)
  {
    bool use_range = false;
    if ((scans[i]->cloud.channels.size() > 0) &&
        (scans[i]->cloud.channels[0].name.compare("range") == 0))
      use_range = true;

    Point start(scans[i]->sensor_origin.translation(0),
                scans[i]->sensor_origin.translation(1));

    bb.min.x = fminf(start.x, bb.min.x);
    bb.min.y = fminf(start.y, bb.min.y);
    bb.max.x = fmaxf(start.x, bb.max.x);
    bb.max.y = fmaxf(start.y, bb.max.y);

    unsigned int npoints = scans[i]->cloud.points.size();
    for (unsigned int j = 0; j < npoints; j++)
    {
      Point end(scans[i]->cloud.points[j].x, scans[i]->cloud.points[j].y);

      if (use_range &&
          (scans[i]->cloud.channels[0].values[j] > scans[i]->max_range))
      {
        // Need to use err mag here, not range mag
        Point err = end - start;
        end = start + scans[i]->max_range * (err / hypotf(err.x, err.y));
      }

      bb.min.x = fminf(end.x, bb.min.x);
      bb.min.y = fminf(end.y, bb.min.y);
      bb.max.x = fmaxf(end.x, bb.max.x);
      bb.max.y = fmaxf(end.y, bb.max.y);
    }
  }

  if (bb != bb_old)
    grid.extend(bb);

  for (unsigned int i = 0; i < nscans; i++)
  {
    Point start(scans[i]->sensor_origin.translation(0),
                scans[i]->sensor_origin.translation(1));

    unsigned int npoints = scans[i]->cloud.points.size();
    for (unsigned int j = 0; j < npoints; j++)
      grid.addRay(
          start,
          Point(scans[i]->cloud.points[j].x, scans[i]->cloud.points[j].y),
          scans[i]->max_range);
  }

  grid.load_time = scans.back()->cloud.header.stamp.toSec();
}
}  // namespace ros
}  // namespace grid2d
}  // namespace map_utils

#endif
