#pragma once

#include <map_utils/Grid3DPoint.h>

namespace map_utils
{
  namespace grid3d
  {
    typedef struct CellValue
    {
      CellValue() : logodds(0.0f) {}
      CellValue(const float& l) : logodds(l) {}
      float logodds;

    } cell_value_t;

    typedef struct GradLogodds : public CellValue
    {
      GradLogodds() : grad(Point(0.0, 0.0, 0.0)) {}
      Point grad;
    } grad_logodds_t;

    typedef struct ROILogodds : public CellValue
    {
    ROILogodds() : roi(false), distance(std::numeric_limits<float>::infinity()) {}
      bool roi;
      float distance;
    } roi_logodds_t;
  }
}
