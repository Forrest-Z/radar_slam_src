#define PCL_NO_PRECOMPIL
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

#include "iostream"
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>

using namespace std;

struct  EIGEN_ALIGN16 ArbePointXYZRGBGeneric
{
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  float range;
  float azimuth;
  float elevation;
  float doppler;
  float power;
  float range_bin;
  float azimuth_bin;
  float elevation_bin;
  float doppler_bin;
  float power_value;
  float timestamp_ms;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT(ArbePointXYZRGBGeneric,           // here we assume a XYZRGB + "detectionData" (as fields)
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, rgb, rgb)
  (float, range, range)
  (float, azimuth, azimuth)
  (float, elevation, elevation)
  (float, doppler, doppler)
  (float, power, power)
  (float, range_bin, range_bin)
  (float, azimuth_bin, azimuth_bin)
  (float, elevation_bin, elevation_bin)
  (float, doppler_bin, doppler_bin)
  (float, power_value, power_value)
  (float, timestamp_ms, timestamp_ms)
)
typedef ArbePointXYZRGBGeneric PointA;
typedef pcl::PointCloud<PointA> PointCloudA;

struct PairHash {
  size_t operator()(const pair<int, int>& p) const {
    std::hash<int> hasher;
    size_t seed = 0;

    seed ^= hasher(p.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hasher(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};


string arbe_origin_topic = "/arbe/rviz/pointcloud";
string arbe_mutipath_topic = "/arbe/feature/mutipath";
string arbe_project_image_topic = "/arbe/feature/project_image/compressed";


float SPACIAL_RES = 0.5;
int ROW = 400;
int COL = 200;
int X_MAX = 20;
float E_MAX = 0.2;
float E_BIAS = -0.1;

int GAUSSIAN_WIDTH = 5;
int GAUSSIAN_HIGH = 5;
int GAUSSIAN_X = 2;
int GAUSSIAN_Y = 2;

int POWER_THRE = 10;


int FENCE_WINDOW_NUMBER = 40;
int FENCE_WINDOW_WIDTH = 5;
int FENCE_WINDOW_HIGH = 5;
int MIN_PIXEL = 10;
int MAX_GAP = 4;

int FENCE_MIN_DISTANCE = 5;