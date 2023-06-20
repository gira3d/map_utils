#include <gtest/gtest.h>

// To read CSV files
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_utils/GeometryUtils.h>
#include <map_utils/Grid3DPoint.h>
#include <map_utils/Grid3D.h>
#include <map_utils/Grid3DROS.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace m3 = map_utils::grid3d;
namespace mr = map_utils::grid3d::ros;

#define GENERATE_TEST_RESULTS 0

#define EPS 1e-05

template<typename T, typename M>
T loadCSV (const std::string & path) {


  std::ifstream indata;
  indata.open(path);
  std::string line; std::vector<M> values;
  uint rows = 0;
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      values.push_back(std::stod(cell));
    }
    ++rows;
  }
  return Eigen::Map<const Eigen::Matrix<typename T::Scalar,
					T::RowsAtCompileTime,
					T::ColsAtCompileTime,
					Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}

// Saves data for map_utils tests
template<typename T>
void save(const std::string& filepath, const Eigen::Matrix<T,-1,-1>& data)
{

  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
					 Eigen::DontAlignCols,
					 ",", "\n");
  std::ofstream file(filepath.c_str());
  file << data.format(CSVFormat);
}

TEST(test_incremental_pointcloud, test_grid_probability)
{
  std::string path = ros::package::getPath("map_utils");

  m3::Point origin(-10,-10,-10);
  float resolution = 0.5;

  float width = 40;
  float height = 40;
  float depth = 40;

  m3::Grid3D<m3::CellValue> grid = m3::Grid3D<m3::CellValue>(width, height, depth, resolution, origin);
  grid.setTrackChanges(true);
  grid.setFreeThreshold(0.13);
  grid.setOccupancyThreshold(0.7);
  grid.setClampingThresholds(0.001, 0.99999);
  grid.setLogOddsHit(0.99);
  grid.setLogOddsMiss(0.33);
  grid.setBlockSize(30);
  grid.setLockAtMaxClamp(false);

  Eigen::MatrixXf pts = loadCSV<Eigen::MatrixXf, float>
    (path + "/tests/data/test_incremental_pointcloud.txt").transpose();

  Eigen::Affine3f Tbc = Eigen::Affine3f::Identity();
  Tbc.translation() = Eigen::Vector3f(0,0,0);
  Tbc.linear() = (Eigen::AngleAxisf(-1.5708, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(-1.5708, Eigen::Vector3f::UnitX()))
                     .toRotationMatrix();

  sensor_msgs::PointCloud in;
  in.points.reserve(pts.rows());

  for (int i = 0; i < pts.cols(); ++i)
  {
    Eigen::Vector3f pc = pts.col(i);
    Eigen::Vector3f pb = Tbc * pc;

    geometry_msgs::Point32 msg;
    msg.x = pb(0);
    msg.y = pb(1);
    msg.z = pb(2);

    in.points.push_back(msg);
  }

  sensor_msgs::PointCloud::ConstPtr in_const =
    sensor_msgs::PointCloud::ConstPtr(new sensor_msgs::PointCloud(in));

  // add registered pointcloud

  gu::Transform3 pose;
  geometry_msgs::Pose p = gr::toPose(pose);

  mr::RegisteredPointCloud::Ptr rpts =
      mr::toRegisteredPointCloud(p, in_const, 5);
  mr::RegisteredPointCloud::ConstPtr rpts_const =
      boost::const_pointer_cast<mr::RegisteredPointCloud>(rpts);
  mr::addRegisteredPointCloud(rpts_const, grid);

  sensor_msgs::PointCloud::Ptr output_pts =
    mr::toIncrementalPointCloud(grid, "/world");

  #if GENERATE_TEST_RESULTS
  Eigen::MatrixXf correct = Eigen::MatrixXf::Zero(output_pts->points.size(), 4);
  #else
  Eigen::MatrixXf correct = loadCSV<Eigen::MatrixXf, float>
    (path + "/tests/data/test_incremental_pointcloud_output.txt");
  ASSERT_EQ(output_pts->points.size(), correct.rows());
  #endif

  for (unsigned i = 0; i < output_pts->points.size(); ++i)
  {
    #if GENERATE_TEST_RESULTS
    correct.row(i) << output_pts->points[i].x, output_pts->points[i].y,
      output_pts->points[i].z, grid.probability(output_pts->channels[0].values[i]);
    #else
    ASSERT_LT(std::abs(output_pts->points[i].x - correct(i,0)), EPS);
    ASSERT_LT(std::abs(output_pts->points[i].y - correct(i,1)), EPS);
    ASSERT_LT(std::abs(output_pts->points[i].z - correct(i,2)), EPS);
    ASSERT_LT(std::abs(grid.probability(output_pts->channels[0].values[i]) - correct(i,3)), EPS);
    #endif
  }

  #if GENERATE_TEST_RESULTS
  save<float>(path + "/tests/data/test_incremental_pointcloud_output.txt", correct);
  #endif
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
