#include <gtest/gtest.h>
#include <map_utils/Grid3D.h>

namespace m3 = map_utils::grid3d;

#define EPS 1e-03

TEST(test_grid3d, inQ)
{
  m3::Point origin = m3::Point(-11, -18, -3.4);
  float resolution = 0.2;
  unsigned int width = 15;
  unsigned int height = 150;
  unsigned int depth = 60;
  float min_clamp = 0.001;
  float max_clamp = 0.9999999;
  float probability_hit = 0.99;
  float probability_miss = 0.33;
  unsigned int block_size = 30;
  bool track_changes = true;
  bool lock_at_max_clamp = false;
  float occupancy_threshold = 0.7;
  float free_threshold = 0.13;

  m3::Grid3D<m3::CellValue> grid = m3::Grid3D<m3::CellValue>(width, height, depth, resolution, origin);
  grid.setClampingThresholds(min_clamp, max_clamp);
  grid.setOccupancyThreshold(occupancy_threshold);
  grid.setFreeThreshold(free_threshold);
  grid.setLogOddsHit(probability_hit);
  grid.setLogOddsMiss(probability_miss);
  grid.setBlockSize(block_size);
  grid.setTrackChanges(track_changes);
  grid.setLockAtMaxClamp(lock_at_max_clamp);

  m3::Point p0(-8.0000, 0.4668, 1.8626);

  ASSERT_TRUE(grid.inQ(p0));
  ASSERT_TRUE(grid.inQ(grid.w2c(p0)));

  //m3::Point p1(-10.9057,-0.2524,2.7607);
  //m3::Point d = p1-p0;
  //m3::Point p2 = p0 + (grid.resolution/2)*d;
  //std::vector<m3::Cell> raycells;
  //bool success = grid.getRay(p0, p1, raycells);
  //std::cout << "Beam passed through " << raycells.size() << " cells" << std::endl;

  //m3::Point p1(-10.9057,-0.2524,2.7607);
  //m3::Point d = p1-p0;
  //m3::Point p2 = p0 + (grid.resolution/2)*d;
  //std::vector<m3::Cell> raycells;
  //bool success = grid.getRay(p2, p1, raycells);
  //std::cout << "Beam passed through " << raycells.size() << " cells" << std::endl;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
