#ifndef MAP_UTILS_GRID_2D_CV_H
#define MAP_UTILS_GRID_2D_CV_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#include "Grid2D.h"

namespace map_utils
{
  namespace grid2d
  {
    namespace cv
    {
      inline bool fromImageFile(const std::string& filename,
                                const Point& origin,
                                float resolution,
                                Grid2D& grid)
      {
        ::cv::Mat image = ::cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
        if(!image.data)
        {
          fprintf(stderr, "OccupancyGrid: Could not open or find the image\n");
          return false;
        }
        ::cv::Mat im;
        ::cv::flip(image, im, 0);

        grid.reset();
        grid.resize(Cell(im.rows, im.cols));

        float occupied = grid.logodds(0.99f);
        float free = grid.logodds(0.01f);
        float unknown = grid.logodds(0.5f);

        unsigned int ncells = grid.data.size();
        for (int i = 0; i < ncells; i++)
        {
          switch (im.data[i])
          {
            case 0:
              grid.data[i] = occupied;
              break;
            case 255:
              grid.data[i] = free;
              break;
            default:
              grid.data[i] = unknown;
              break;
          }
        }

        grid.resolution = resolution;
        grid.origin = origin;

        return true;
      }

      inline bool toImageFile(Grid2D::Ptr& grid,
                              const std::string& filename)
      {
        ::cv::Mat im = ::cv::Mat::zeros(grid->height, grid->width, CV_8UC1);

        unsigned int ncells = grid->data.size();
        for (int i = 0; i < ncells; i++)
        {
          if (grid->occupiedQ(i))
            im.data[i] = 0;
          else if (grid->freeQ(i))
            im.data[i] = 255;
          else
            im.data[i] = 100;
        }

        ::cv::Mat im2;
        ::cv::flip(im, im2, 0);
        ::cv::imwrite(filename, im2);

        std::cout << "Saved file to : " << filename << std::endl;

        return true;
      }

      inline Grid2D::Ptr createDistanceField(const Grid2D::ConstPtr& grid)
      {
        unsigned int ndata = grid->data.size();

        ::cv::Mat obstacles(grid->height, grid->width, CV_8UC1, 1);
        for (unsigned int i = 0; i < ndata; i++)
          if (grid->occupiedQ(i))
            obstacles.data[i] = 0;

        // Compute the submap distance transform
        ::cv::Mat dist(grid->height, grid->width, CV_32FC1, 1);
        ::cv::distanceTransform(obstacles, dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);

        Grid2D::Ptr out = Grid2D::Ptr(new Grid2D);
        out->data.resize(ndata);
        out->resolution = grid->resolution;
        out->width = grid->width;
        out->height = grid->height;
        out->load_time = grid->load_time;
        out->origin = grid->origin;
        for (unsigned int r = 0; r < out->height; ++r)
        {
          const float* di = dist.ptr<float>(r);
          for (unsigned int c = 0; c < out->width; ++c)
            out->data[r*out->width + c] = grid->resolution*di[c];
        }

        return out;
      }

      inline void createDistanceField(const Grid2D& grid,
                                      const BoundingBox& bbx,
                                      Grid2D& out)
      {
        if (!grid.inQ(bbx))
        {
          fprintf(stderr, "[Grid2DCV]: bbx not in grid\n");
          return;
        }

        Cell cl = grid.w2c(bbx.min);
        Cell cu = grid.w2c(bbx.max);

        unsigned int dr = cu.row - cl.row + 1;
        unsigned int dc = cu.col - cl.col + 1;

        ::cv::Mat obstacles(dr, dc, CV_8UC1, 1);
        for (unsigned int r = 0; r < dr; r++)
          for (unsigned int c = 0; c < dc; c++)
            if (grid.occupiedQ(Cell(cl.row + r, cl.col + c)))
              obstacles.data[c + r*dc] = 0;

        // Compute the submap distance transform
        ::cv::Mat dist(dr, dc, CV_32FC1, 1);
        ::cv::distanceTransform(obstacles, dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);

        out.reset();
        out.data.resize(dc*dr);
        out.resolution = grid.resolution;
        out.width = dc;
        out.height = dr;
        out.load_time = grid.load_time;
        out.origin = grid.c2w(cl);
        for (unsigned int r = 0; r < out.height; ++r)
        {
          const float* di = dist.ptr<float>(r);
          for (unsigned int c = 0; c < out.width; ++c)
            out.data[r*out.width + c] = grid.resolution*di[c];
        }
      }
    }
  }
}
#endif
