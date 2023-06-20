#include <gtest/gtest.h>

// To read CSV files
#include <Eigen/Dense>

#include <ros/ros.h>
#include<ros/package.h>
#include <geometry_utils/GeometryUtils.h>
#include <map_utils/Grid3DPoint.h>
#include <map_utils/Grid3D.h>

namespace gu = geometry_utils;
namespace m3 = map_utils::grid3d;

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

TEST(test_intersections, min_max_ray)
{
  std::string path = ros::package::getPath("map_utils");

  m3::Point origin(-2,-2,-2);
  float resolution = 0.2;

  float width = 20;
  float height = 20;
  float depth = 20;

  m3::Grid3D<m3::CellValue> og = m3::Grid3D<m3::CellValue>(width, height, depth, resolution, origin);
  m3::bbx_t bbx = og.getBBX();

  m3::Point min = bbx.min;
  m3::Point max = bbx.max;

  std::vector<m3::Cell> cells;

  og.getRay(min, max, cells);

  #if GENERATE_TEST_RESULTS
  Eigen::MatrixXf correct = Eigen::MatrixXf::Zero(cells.size(), 3);
  #else
  Eigen::MatrixXf correct = loadCSV<Eigen::MatrixXf, float>
    (path + "/tests/data/test_intersections_min_max_ray.txt");
  #endif

  for (unsigned i = 0; i < cells.size(); ++i)
  {
    unsigned int idx = og.getIndex(cells[i]);
    m3::Point p = og.getPoint(idx);

    #if GENERATE_TEST_RESULTS
    correct.row(i) << p.x, p.y, p.z;
    #else
    ASSERT_LT(std::abs(p.x - correct(i,0)), EPS);
    ASSERT_LT(std::abs(p.y - correct(i,1)), EPS);
    ASSERT_LT(std::abs(p.z - correct(i,2)), EPS);
    #endif
  }

  #if GENERATE_TEST_RESULTS
  save<float>(path + "/tests/data/test_intersections_min_max_ray.txt", correct);
  #endif
}

TEST(test_intersections, x_constant_ray)
{
  std::string path = ros::package::getPath("map_utils");

  m3::Point origin(-2,-2,-2);
  float resolution = 0.5;

  float width = 20;
  float height = 20;
  float depth = 20;

  m3::Grid3D<m3::CellValue> og = m3::Grid3D<m3::CellValue>(width, height, depth, resolution, origin);

  m3::Point min(-1,-1,-1);
  m3::Point max(-1,1,1);

  std::vector<m3::Cell> cells;

  og.getRay(min, max, cells);

  #if GENERATE_TEST_RESULTS
  Eigen::MatrixXf correct = Eigen::MatrixXf::Zero(cells.size(), 3);
  #else
  Eigen::MatrixXf correct = loadCSV<Eigen::MatrixXf, float>
    (path + "/tests/data/test_intersections_x_constant_ray.txt");
  #endif

  for (unsigned i = 0; i < cells.size(); ++i)
  {
    unsigned int idx = og.getIndex(cells[i]);
    m3::Point p = og.getPoint(idx);

    #if GENERATE_TEST_RESULTS
    correct.row(i) << p.x, p.y, p.z;
    #else
    ASSERT_LT(std::abs(p.x - correct(i,0)), EPS);
    ASSERT_LT(std::abs(p.y - correct(i,1)), EPS);
    ASSERT_LT(std::abs(p.z - correct(i,2)), EPS);
    #endif
  }

  #if GENERATE_TEST_RESULTS
  save<float>(path + "/tests/data/test_intersections_x_constant_ray.txt", correct);
  #endif
}

TEST(test_intersections, y_constant_ray)
{
  std::string path = ros::package::getPath("map_utils");

  m3::Point origin(-4,-4,-4);
  float resolution = 0.1;

  float width = 30;
  float height = 30;
  float depth = 30;

  m3::Grid3D<m3::CellValue> og = m3::Grid3D<m3::CellValue>(width, height, depth, resolution, origin);

  m3::Point min(-1.05,-1.05,-1.05);
  m3::Point max(-2,-1,-2);

  std::vector<m3::Cell> cells;

  og.getRay(min, max, cells);

  #if GENERATE_TEST_RESULTS
  Eigen::MatrixXf correct = Eigen::MatrixXf::Zero(cells.size(), 3);
  #else
  Eigen::MatrixXf correct = loadCSV<Eigen::MatrixXf, float>
    (path + "/tests/data/test_intersections_y_constant_ray.txt");
  #endif

  for (unsigned i = 0; i < cells.size(); ++i)
  {
    unsigned int idx = og.getIndex(cells[i]);
    m3::Point p = og.getPoint(idx);

    #if GENERATE_TEST_RESULTS
    correct.row(i) << p.x, p.y, p.z;
    #else
    ASSERT_LT(std::abs(p.x - correct(i,0)), EPS);
    ASSERT_LT(std::abs(p.y - correct(i,1)), EPS);
    ASSERT_LT(std::abs(p.z - correct(i,2)), EPS);
    #endif
  }
  #if GENERATE_TEST_RESULTS
  save<float>(path + "/tests/data/test_intersections_y_constant_ray.txt", correct);
  #endif
}

TEST(test_intersections, z_constant_ray)
{
  std::string path = ros::package::getPath("map_utils");

  m3::Point origin(-4,-4,-4);
  float resolution = 0.05;

  float width = 60;
  float height = 60;
  float depth = 20;

  m3::Grid3D<m3::CellValue> og = m3::Grid3D<m3::CellValue>(width, height, depth, resolution, origin);

  m3::Point min(-3.9,-3.9,-3.9);
  m3::Point max(-2,-2,-3.9);

  std::vector<m3::Cell> cells;

  og.getRay(min, max, cells);

  #if GENERATE_TEST_RESULTS
  Eigen::MatrixXf correct = Eigen::MatrixXf::Zero(cells.size(), 3);
  #else
  Eigen::MatrixXf correct = loadCSV<Eigen::MatrixXf, float>
    (path + "/tests/data/test_intersections_z_constant_ray.txt");
  #endif

  for (unsigned i = 0; i < cells.size(); ++i)
  {
    unsigned int idx = og.getIndex(cells[i]);
    m3::Point p = og.getPoint(idx);

    #if GENERATE_TEST_RESULTS
    correct.row(i) << p.x, p.y, p.z;
    #else
    ASSERT_LT(std::abs(p.x - correct(i,0)), EPS);
    ASSERT_LT(std::abs(p.y - correct(i,1)), EPS);
    ASSERT_LT(std::abs(p.z - correct(i,2)), EPS);
    #endif
  }
  #if GENERATE_TEST_RESULTS
  save<float>(path + "/tests/data/test_intersections_z_constant_ray.txt", correct);
  #endif
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
