#ifndef MAP_UTILS_GRID_3D_ROS_H
#define MAP_UTILS_GRID_3D_ROS_H

#include <geometry_utils/GeometryUtilsROS.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <map_msgs/PointCloud2Update.h>
#include <map_utils/Grid3DMsg.h>
#include "Grid3D.h"
#include "Grid3DCellValue.h"

namespace map_utils
{
  namespace grid3d
  {
    namespace ros
    {
      template <typename T = CellValue,
                bool (*PublishPointQ)(const Grid3D<T>&, const Cell&),
                void (*PublishPointF)(const Grid3D<T>&, const Cell&,
                                      sensor_msgs::PointCloud::Ptr&)>
      inline void toPointCloud(const Grid3D<T>& grid,
                               sensor_msgs::PointCloud::Ptr& msg)
      {
        float g2 = 0.5*grid.resolution;

        for (unsigned int r = 0; r < grid.height; r++)
          for (unsigned int c = 0; c < grid.width; c++)
            for (unsigned int s = 0; s < grid.depth; s++)
            {
              Cell cell(r, c, s);
              if (!PublishPointQ(grid, cell))
                continue;

              Point pw = grid.c2w(cell);

              geometry_msgs::Point32 p;
              // Compensate for middle of point vs lower-left corner
              p.x = pw.x + g2;
              p.y = pw.y + g2;
              p.z = pw.z + g2;

              msg->points.push_back(p);
              PublishPointF(grid, cell, msg);
            }
      }

      template <typename T = CellValue>
      inline void publish_logodds(const Grid3D<T>& grid, const Cell& cell,
                                  sensor_msgs::PointCloud::Ptr& msg)
      {
        for (size_t i = 0; i < msg->channels.size(); i++)
        {
          if (msg->channels[i].name == "logodds")
          {
            msg->channels[i].values.push_back(grid.get(cell).logodds);
            break;
          }
        }
      };

      // Convert a grid to a pointcloud and filter points using a function of
      // the grid and cell index.
      //
      // This method allows for general filtering strategies such as for
      // publishing frontier cells.
      //
      // PublishPointQ should return true if a given cell and its value should
      // be included in the output pointcloud.
      template< typename T = CellValue, bool (*PublishPointQ)(const Grid3D<T>&, const Cell&)>
      inline sensor_msgs::PointCloud::Ptr
      toPointCloud(const Grid3D<T>& grid, const std::string& frame_id)
      {
        sensor_msgs::PointCloud::Ptr msg =
          sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud());

        msg->header.stamp = ::ros::Time(grid.load_time);
        msg->header.frame_id = frame_id;
        msg->points.reserve(grid.data.size());
        msg->channels.resize(1);
        msg->channels[0].name = "logodds";
        msg->channels[0].values.reserve(grid.data.size());

        float g2 = 0.5*grid.resolution;

        for (unsigned int r = 0; r < grid.height; r++)
          for (unsigned int c = 0; c < grid.width; c++)
            for (unsigned int s = 0; s < grid.depth; s++)
            {
              Cell cell(r, c, s);
              if (!PublishPointQ(grid, cell))
                continue;

              Point pw = grid.c2w(cell);

              geometry_msgs::Point32 p;
              // Compensate for middle of point vs lower-left corner
              p.x = pw.x + g2;
              p.y = pw.y + g2;
              p.z = pw.z + g2;

              msg->points.push_back(p);
              msg->channels[0].values.push_back(grid.get(cell).logodds);
            }

        return msg;
      }

      // This helper function turns a filtering function (the template argument)
      // of cell values into a general function of grids and cells as required
      // by the more general form of toPointCloud.
      template< typename T = CellValue, bool (*PublishQByValue)(double)>
      inline bool publish_by_cell(const Grid3D<T>& grid, const Cell& cell)
      {
        return PublishQByValue(grid.get(cell).logodds);
      };

      // Convert a grid to a pointcloud and filter by values
      //
      // PublishQByValue takes a value stored at a cell as input and returns
      // true if that cell and its value should be included in the output
      // pointcloud
      template< typename T = CellValue, bool (*PublishQByValue)(double)>
      inline sensor_msgs::PointCloud::Ptr
      toPointCloud(const Grid3D<T>& grid, const std::string& frame_id)
      {
        return toPointCloud<T, publish_by_cell<T, PublishQByValue>>(grid, frame_id);
      }

      template< typename T = CellValue >
      inline sensor_msgs::PointCloud::Ptr
      toPointCloud(const Grid3D<T>& grid, const std::string& frame_id, bool include_all = false)
      {
        sensor_msgs::PointCloud::Ptr msg =
          sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud());

        msg->header.stamp = ::ros::Time(grid.load_time);
        msg->header.frame_id = frame_id;
        msg->points.reserve(grid.data.size());
        msg->channels.resize(1);
        msg->channels[0].name = "logodds";
        msg->channels[0].values.reserve(grid.data.size());

        float g2 = 0.5*grid.resolution;

        for (unsigned int r = 0; r < grid.height; r++)
          for (unsigned int c = 0; c < grid.width; c++)
            for (unsigned int s = 0; s < grid.depth; s++)
            {
              Cell pc(r, c, s);
              if (!include_all && grid.unknownQ(pc))
                continue;

              Point pw = grid.c2w(pc);

              geometry_msgs::Point32 p;
              // Compensate for middle of point vs lower-left corner
              p.x = pw.x + g2;
              p.y = pw.y + g2;
              p.z = pw.z + g2;

              msg->points.push_back(p);
              msg->channels[0].values.push_back(grid.get(pc).logodds);
            }

        return msg;
      }

      template< typename T = CellValue >
      inline sensor_msgs::PointCloud::Ptr
      toFreePointCloud(const Grid3D<T>& grid, const std::string& frame_id)
      {
        sensor_msgs::PointCloud::Ptr msg =
          sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud());

        msg->header.stamp = ::ros::Time(grid.load_time);
        msg->header.frame_id = frame_id;
        msg->points.reserve(grid.data.size());
        msg->channels.resize(1);
        msg->channels[0].name = "free";
        msg->channels[0].values.reserve(grid.data.size());

        float g2 = 0.5*grid.resolution;

        for (unsigned int r = 0; r < grid.height; r++)
          for (unsigned int c = 0; c < grid.width; c++)
            for (unsigned int s = 0; s < grid.depth; s++)
            {
              Cell pc(r, c, s);
              if (!grid.freeQ(pc))
                continue;

              Point pw = grid.c2w(pc);

              geometry_msgs::Point32 p;
              // Compensate for middle of point vs lower-left corner
              p.x = pw.x + g2;
              p.y = pw.y + g2;
              p.z = pw.z + g2;

              msg->points.push_back(p);
              msg->channels[0].values.push_back(grid.get(pc).logodds);
            }

        return msg;
      }

      template< typename T = CellValue >
      inline sensor_msgs::PointCloud::Ptr
      toOccupiedPointCloud(const Grid3D<T>& grid, const std::string& frame_id)
      {
        sensor_msgs::PointCloud::Ptr msg =
          sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud());

        msg->header.stamp = ::ros::Time(grid.load_time);
        msg->header.frame_id = frame_id;
        msg->points.reserve(grid.data.size());
        msg->channels.resize(1);
        msg->channels[0].name = "occupied";
        msg->channels[0].values.reserve(grid.data.size());

        float g2 = 0.5*grid.resolution;

        for (unsigned int r = 0; r < grid.height; r++)
          for (unsigned int c = 0; c < grid.width; c++)
            for (unsigned int s = 0; s < grid.depth; s++)
            {
              Cell pc(r, c, s);
              if (!grid.occupiedQ(pc))
                continue;

              Point pw = grid.c2w(pc);

              geometry_msgs::Point32 p;
              // Compensate for middle of point vs lower-left corner
              p.x = pw.x + g2;
              p.y = pw.y + g2;
              p.z = pw.z + g2;

              msg->points.push_back(p);
              msg->channels[0].values.push_back(grid.get(pc).logodds);
            }

        return msg;
      }

      template< typename T = CellValue >
      inline sensor_msgs::PointCloud::Ptr
      toUnknownPointCloud(const Grid3D<T>& grid, const std::string& frame_id)
      {
        sensor_msgs::PointCloud::Ptr msg =
          sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud());

        msg->header.stamp = ::ros::Time(grid.load_time);
        msg->header.frame_id = frame_id;
        msg->points.reserve(grid.data.size());
        msg->channels.resize(1);
        msg->channels[0].name = "unknown";
        msg->channels[0].values.reserve(grid.data.size());

        float g2 = 0.5*grid.resolution;

        for (unsigned int r = 0; r < grid.height; r++)
          for (unsigned int c = 0; c < grid.width; c++)
            for (unsigned int s = 0; s < grid.depth; s++)
            {
              Cell pc(r, c, s);
              if (!grid.unknownQ(pc))
                continue;

              Point pw = grid.c2w(pc);

              geometry_msgs::Point32 p;
              // Compensate for middle of point vs lower-left corner
              p.x = pw.x + g2;
              p.y = pw.y + g2;
              p.z = pw.z + g2;

              msg->points.push_back(p);
              msg->channels[0].values.push_back(grid.get(pc).logodds);
            }

        return msg;
      }

      template< typename T = CellValue >
      inline sensor_msgs::PointCloud::Ptr
      toIncrementalPointCloud(const Grid3D<T>& grid, const std::string& frame_id)
      {
        if (!grid.track_changes)
          ROS_WARN("[Grid3D] toIncrementalPointCloud - not tracking");

        unsigned int nchanges = grid.changes.size();

        sensor_msgs::PointCloud::Ptr msg =
          sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud());

        msg->header.stamp = ::ros::Time(grid.load_time);
        msg->header.frame_id = frame_id;
        msg->points.reserve(nchanges);
        msg->channels.resize(1);
        msg->channels[0].name = "logodds";
        msg->channels[0].values.reserve(nchanges);

        float g2 = 0.5f*grid.resolution;

        for (typename Grid3D<T>::changeset_t::const_iterator it = grid.changes.begin();
             it != grid.changes.end(); ++it)
        {
          Point pw = grid.getPoint(*it);

          geometry_msgs::Point32 p;
          // Compensate for middle of point vs lower-left corner
          p.x = pw.x + g2;
          p.y = pw.y + g2;
          p.z = pw.z + g2;

          msg->points.push_back(p);
          msg->channels[0].values.push_back(grid.get(*it).logodds);
        }

        return msg;
      }

      template< typename T = CellValue >
      inline map_utils::Grid3DMsg::Ptr
      toGrid3DMsg(const Grid3D<T>& grid, const std::string& frame_id)
      {
        if (!grid.track_changes)
          ROS_WARN("[Grid3D] toIncrementalPointCloud - not tracking");

        unsigned int nchanges = grid.changes.size();

        map_utils::Grid3DMsg::Ptr msg =
          map_utils::Grid3DMsg::Ptr(new map_utils::Grid3DMsg());

        msg->header.stamp = ::ros::Time(grid.load_time);
        msg->header.frame_id = frame_id;
        msg->logodds.reserve(nchanges);
        msg->idxs.reserve(nchanges);
	msg->width = grid.width;
	msg->height = grid.height;
	msg->depth = grid.depth;
	msg->origin.x = grid.origin.x;
	msg->origin.y = grid.origin.y;
	msg->origin.z = grid.origin.z;

        for (typename Grid3D<T>::changeset_t::const_iterator it = grid.changes.begin();
             it != grid.changes.end(); ++it)
        {
	  msg->logodds.push_back(grid.get(*it).logodds);
	  msg->idxs.push_back(*it);
        }

        return msg;
      }

      template< typename T = CellValue >
      inline void
      fromGrid3DMsg(Grid3D<T>& grid,
		    const map_utils::Grid3DMsg& msg,
		    const bool& reset_changeset = true)
      {

	if ( (msg.logodds.size() != msg.idxs.size()) )
	{
	  ROS_ERROR("[Grid3D] fromGrid3DMsg - logodds and idxs sizes don't match");
	  return;
	}

	if ( (msg.origin.x != grid.origin.x) ||
	     (msg.origin.y != grid.origin.y) ||
	     (msg.origin.z != grid.origin.z) ||
	     (msg.width != grid.width) ||
	     (msg.height != grid.height) ||
	     (msg.depth != grid.depth) )
	{
	  ROS_ERROR("[Grid3D] fromGrid3DMsg - grid dimensions do not match");
	  return;
	}

        for (unsigned int i = 0; i < msg.logodds.size(); ++i)
	{
	  typename map_utils::grid3d::Grid3D<T>::CELL_VALUE v;
	  v.logodds = msg.logodds[i];
	  grid.set(msg.idxs[i], v);
	}

	if (reset_changeset)
	  grid.resetChangeSet();
      }

      struct RegisteredPointCloud
      {
        typedef boost::shared_ptr<RegisteredPointCloud> Ptr;
        typedef boost::shared_ptr<const RegisteredPointCloud> ConstPtr;

        geometry_utils::Transform3 sensor_origin;
        sensor_msgs::PointCloud cloud;
        float max_range;
      };

      inline RegisteredPointCloud::Ptr
      toRegisteredPointCloud(const geometry_msgs::Pose& pose,
                             const sensor_msgs::PointCloud::ConstPtr& msg,
                             float max_range)
      {
        RegisteredPointCloud::Ptr p(new RegisteredPointCloud);

        // Transform to grid frame
        p->sensor_origin = geometry_utils::ros::fromROS(pose);

        p->max_range = max_range;

        p->cloud.header = msg->header;
        p->cloud.points.resize(msg->points.size());

        bool use_channels = false;
        if (msg->channels.size() > 0)
        {
          use_channels = true;
          p->cloud.channels.resize(1);
          p->cloud.channels[0].name = msg->channels[0].name;
          p->cloud.channels[0].values.resize(msg->channels[0].values.size());
        }

        unsigned int npoints = msg->points.size();
        for (unsigned int i = 0; i < npoints; i++)
        {
          geometry_utils::Vec3 pw =
            p->sensor_origin*geometry_utils::ros::fromROS(msg->points[i]);
          p->cloud.points[i] = geometry_utils::ros::toPoint32(pw);
          if (use_channels)
            p->cloud.channels[0].values[i] = msg->channels[0].values[i];
        }

        return p;
      }

      template< typename T = CellValue >
      inline void
      addRegisteredPointCloud(const RegisteredPointCloud::ConstPtr& cloud, Grid3D<T>& grid)
      {
        bool use_range = false;
        if ((cloud->cloud.channels.size() > 0) &&
            (cloud->cloud.channels[0].name.compare("range") == 0))
          use_range = true;

        // Get the scan min and max points
        bbx_t bb = grid.getBBX();
        bbx_t bb_old = bb;

        Point start(cloud->sensor_origin.translation(0),
                    cloud->sensor_origin.translation(1),
                    cloud->sensor_origin.translation(2));

        // Ensure that sensor origin is in the map
        bb.min.x = fminf(start.x, bb.min.x);
        bb.min.y = fminf(start.y, bb.min.y);
        bb.min.z = fminf(start.z, bb.min.z);
        bb.max.x = fmaxf(start.x, bb.max.x);
        bb.max.y = fmaxf(start.y, bb.max.y);
        bb.max.z = fmaxf(start.z, bb.max.z);

        unsigned int npoints = cloud->cloud.points.size();
        for (unsigned int i = 0; i < npoints; i++)
        {
          Point end(cloud->cloud.points[i].x,
                    cloud->cloud.points[i].y,
                    cloud->cloud.points[i].z);

          if (use_range && (cloud->cloud.channels[0].values[i] > cloud->max_range))
          {
            Point err = end - start;
            end =
              start + cloud->max_range*(err/sqrtf(err.x*err.x + err.y*err.y + err.z*err.z));
          }

          bb.min.x = fminf(end.x, bb.min.x);
          bb.min.y = fminf(end.y, bb.min.y);
          bb.min.z = fminf(end.z, bb.min.z);
          bb.max.x = fmaxf(end.x, bb.max.x);
          bb.max.y = fmaxf(end.y, bb.max.y);
          bb.max.z = fmaxf(end.z, bb.max.z);
        }

        if (bb != bb_old)
          grid.extend(bb);

        for (unsigned int i = 0; i < npoints; i++)
          grid.addRay(start,
                      Point(cloud->cloud.points[i].x,
                            cloud->cloud.points[i].y,
                            cloud->cloud.points[i].z),
                      cloud->max_range);

        grid.load_time =
            std::max(grid.load_time, cloud->cloud.header.stamp.toSec());
      }

      template< typename T = CellValue >
      inline void
      addRegisteredPointClouds(const std::vector<RegisteredPointCloud::ConstPtr>& clouds,
                               Grid3D<T>& grid)
      {
        unsigned int n = clouds.size();
        if (n == 0)
          return;

        // Get the scan min and max points
        bbx_t bb = grid.getBBX();
        bbx_t bb_old(bb);

        for (unsigned int i = 0; i < n; i++)
        {
          bool use_range = false;
          if ((clouds[i]->cloud.channels.size() > 0) &&
              (clouds[i]->cloud.channels[0].name.compare("range") == 0))
            use_range = true;

          Point start(clouds[i]->sensor_origin.translation(0),
                      clouds[i]->sensor_origin.translation(1),
                      clouds[i]->sensor_origin.translation(2));

          // Ensure that sensor origin is in the map
          bb.min.x = fminf(start.x, bb.min.x);
          bb.min.y = fminf(start.y, bb.min.y);
          bb.min.z = fminf(start.z, bb.min.z);
          bb.max.x = fmaxf(start.x, bb.max.x);
          bb.max.y = fmaxf(start.y, bb.max.y);
          bb.max.z = fmaxf(start.z, bb.max.z);

          unsigned int npoints = clouds[i]->cloud.points.size();
          for (unsigned int j = 0; j < npoints; j++)
          {
            Point end(clouds[i]->cloud.points[j].x,
                      clouds[i]->cloud.points[j].y,
                      clouds[i]->cloud.points[j].z);

            if (use_range && (clouds[i]->cloud.channels[0].values[j] > clouds[i]->max_range))
            {
              Point err = end - start;
              end = start + clouds[i]->max_range*(err/sqrtf(err.x*err.x + err.y*err.y + err.z*err.z));
            }

            bb.min.x = fminf(end.x, bb.min.x);
            bb.min.y = fminf(end.y, bb.min.y);
            bb.min.z = fminf(end.z, bb.min.z);
            bb.max.x = fmaxf(end.x, bb.max.x);
            bb.max.y = fmaxf(end.y, bb.max.y);
            bb.max.z = fmaxf(end.z, bb.max.z);
          }
        }

        if (bb != bb_old)
          grid.extend(bb);

        for (unsigned int i = 0; i < n; i++)
        {
          Point start(clouds[i]->sensor_origin.translation(0),
                      clouds[i]->sensor_origin.translation(1),
                      clouds[i]->sensor_origin.translation(2));

          unsigned int npoints = clouds[i]->cloud.points.size();
          for (unsigned int j = 0; j < npoints; j++)
            grid.addRay(start,
                        Point(clouds[i]->cloud.points[j].x,
                              clouds[i]->cloud.points[j].y,
                              clouds[i]->cloud.points[j].z),
                        clouds[i]->max_range);
        }

        grid.load_time =
            std::max(grid.load_time, clouds.back()->cloud.header.stamp.toSec());
      }

      template< typename T = CellValue >
      inline void addPointCloud(const sensor_msgs::PointCloud::ConstPtr& cloud, Grid3D<T>& grid)
      {
        // Get the scan min and max points
        bbx_t bb = grid.getBBX();
        bbx_t bb_old = bb;

        unsigned int npoints = cloud->points.size();
        for (unsigned int i = 0; i < npoints; i++)
        {
          bb.min.x = fminf(cloud->points[i].x, bb.min.x);
          bb.min.y = fminf(cloud->points[i].y, bb.min.y);
          bb.min.z = fminf(cloud->points[i].z, bb.min.z);
          bb.max.x = fmaxf(cloud->points[i].x, bb.max.x);
          bb.max.y = fmaxf(cloud->points[i].y, bb.max.y);
          bb.max.z = fmaxf(cloud->points[i].z, bb.max.z);
        }

        if (bb != bb_old)
          grid.extend(bb);

        for (unsigned int i = 0; i < npoints; i++)
	{
	  typename map_utils::grid3d::Grid3D<T>::CELL_VALUE v;
	  v.logodds = Grid3D<T>::OCCUPIED;
          grid.set(Point(cloud->points[i].x,
                         cloud->points[i].y,
                         cloud->points[i].z), v);
	}

        grid.load_time = std::max(grid.load_time, cloud->header.stamp.toSec());
      }

      template< typename T = CellValue >
      inline void addPointClouds(const std::vector<sensor_msgs::PointCloud::ConstPtr>& clouds,
                                 Grid3D<T>& grid)
      {
        unsigned int n = clouds.size();
        if (n == 0)
          return;

        // Get the scan min and max points
        bbx_t bb = grid.getBBX();
        bbx_t bb_old = bb;

        for (unsigned int i = 0; i < n; i++)
        {
          unsigned int npoints = clouds[i]->points.size();
          for (unsigned int j = 0; j < npoints; j++)
          {
            bb.min.x = fminf(clouds[i]->points[j].x, bb.min.x);
            bb.min.y = fminf(clouds[i]->points[j].y, bb.min.y);
            bb.min.z = fminf(clouds[i]->points[j].z, bb.min.z);
            bb.max.x = fmaxf(clouds[i]->points[j].x, bb.max.x);
            bb.max.y = fmaxf(clouds[i]->points[j].y, bb.max.y);
            bb.max.z = fmaxf(clouds[i]->points[j].z, bb.max.z);
          }
        }

        if (bb != bb_old)
          grid.extend(bb);

        for (unsigned int i = 0; i < n; i++)
        {
          unsigned int npoints = clouds[i]->points.size();
          for (unsigned int j = 0; j < npoints; j++)
	  {
	    typename map_utils::grid3d::Grid3D<T>::CELL_VALUE v;
	    v.logodds = Grid3D<T>::OCCUPIED;
            grid.set(Point(clouds[i]->points[j].x,
                           clouds[i]->points[j].y,
                           clouds[i]->points[j].z), v);
	  }
        }

        grid.load_time =
            std::max(grid.load_time, clouds.back()->header.stamp.toSec());
      }

      template< typename T = CellValue >
      inline void addIncrementalPointCloud(const sensor_msgs::PointCloud::ConstPtr& cloud,
                                           Grid3D<T>& grid)
      {
        if ((cloud->channels.size() != 1) || (cloud->channels[0].name != "logodds"))
        {
          ROS_WARN("[Grid3D] addIncrementalPointCloud - improperly formatted cloud");
          return;
        }

        bbx_t bb = grid.getBBX();
        bbx_t bb_old = bb;

        unsigned int npoints = cloud->points.size();
        for (unsigned int i = 0; i < npoints; i++)
        {
          bb.min.x = fminf(cloud->points[i].x, bb.min.x);
          bb.min.y = fminf(cloud->points[i].y, bb.min.y);
          bb.min.z = fminf(cloud->points[i].z, bb.min.z);
          bb.max.x = fmaxf(cloud->points[i].x, bb.max.x);
          bb.max.y = fmaxf(cloud->points[i].y, bb.max.y);
          bb.max.z = fmaxf(cloud->points[i].z, bb.max.z);
        }

        if (bb != bb_old)
          grid.extend(bb);

        for (unsigned int i = 0; i < npoints; i++)
	{
	  typename map_utils::grid3d::Grid3D<T>::CELL_VALUE v;
	  v.logodds = cloud->channels[0].values[i];
          grid.set(Point(cloud->points[i].x,
                         cloud->points[i].y,
                         cloud->points[i].z),
		   v);
	}

        grid.load_time = std::max(grid.load_time, cloud->header.stamp.toSec());
      }

      template< typename T = CellValue >
      inline void
      addIncrementalPointClouds(const std::vector<sensor_msgs::PointCloud::ConstPtr>& clouds,
                                Grid3D<T>& grid)
      {
        unsigned int n = clouds.size();
        if (n == 0)
          return;

        for (unsigned int i = 0; i < n; i++)
        {
          if ((clouds[i]->channels.size() != 1) ||
              (clouds[i]->channels[0].name != "logodds"))
          {
            ROS_WARN("[Grid3D] addIncrementalPointCloud - improperly formatted cloud");
            return;
          }
        }

        bbx_t bb = grid.getBBX();
        bbx_t bb_old = bb;

        for (unsigned int i = 0; i < n; i++)
        {
          unsigned int npoints = clouds[i]->points.size();
          for (unsigned int j = 0; j < npoints; j++)
          {
            bb.min.x = fminf(clouds[i]->points[j].x, bb.min.x);
            bb.min.y = fminf(clouds[i]->points[j].y, bb.min.y);
            bb.min.z = fminf(clouds[i]->points[j].z, bb.min.z);
            bb.max.x = fmaxf(clouds[i]->points[j].x, bb.max.x);
            bb.max.y = fmaxf(clouds[i]->points[j].y, bb.max.y);
            bb.max.z = fmaxf(clouds[i]->points[j].z, bb.max.z);
          }
        }

        if (bb != bb_old)
          grid.extend(bb);

        for (unsigned int i = 0; i < n; i++)
        {
          unsigned int npoints = clouds[i]->points.size();
          for (unsigned int j = 0; j < npoints; j++)
	  {
	    typename map_utils::grid3d::Grid3D<T>::CELL_VALUE v;
	    v.logodds = clouds[i]->channels[0].values[j];
            grid.set(Point(clouds[i]->points[j].x,
                           clouds[i]->points[j].y,
                           clouds[i]->points[j].z),
		     v);
	  }
        }

        grid.load_time =
            std::max(grid.load_time, clouds.back()->header.stamp.toSec());
      }
    }
  }
}

#endif
