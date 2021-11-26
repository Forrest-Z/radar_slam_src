#define PCL_NO_PRECOMPIL
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>

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

typedef struct
{
  int sum_power;
  vector<int> index;
  int max_range_bin;
  int min_range_bin;
  int mean_range_bin;
  int std_range_bin;

}ProjectInfo;

// int OS_CFAR_RADIUS = 4;
// int OS_CFAR_RADIUS_RANGE = 13;
// int OS_CFAR_RADIUS_AZIMUTH = 2;
// int MIN_OS_CFAR_NUMBER = 33;
// int MAX_OS_CFAR_NUMBER = 66;
// float OS_CFAR_A = 1.1;
// int MAX_ELEVATION_NUMBER = 10;
// int MAX_ELEVATION_RANGE = 24;
// int ROW = 512;
// int COL = 128;

int OS_CFAR_RADIUS = 4;
// int OS_CFAR_RADIUS_RANGE = 29;
int OS_CFAR_RADIUS_RANGE = 2;
int OS_CFAR_RADIUS_AZIMUTH = 8;
int MIN_OS_CFAR_NUMBER = 32;
int MAX_OS_CFAR_NUMBER = 66;
float OS_CFAR_A = 3;
int MAX_ELEVATION_NUMBER = 10;
int MAX_ELEVATION_RANGE = 24;
int ROW = 32;
int COL = 128;

float MAX_SHANG = 2000;
int RANGE_GROUP_WIDTH = 10;
int DOPPELR_GROUP_WIDTH = 10;
int ELEVATION_GROUP_WIDTH = 1;
int MIN_SHANG = 10000;

ros::Subscriber arbe_origin_sub;
ros::Publisher arbe_os_cfar_pub;
ros::Publisher arbe_project_image_pub;
std_msgs::Header  cloud_header;
PointCloudA::Ptr arbe_origin_pcl;
PointCloudA::Ptr arbe_os_cfar_pcl;
cv::Mat arbe_project_image;
sensor_msgs::CompressedImage arbe_project_image_msg;

string arbe_origin_topic = "/arbe/rviz/pointcloud";
string arbe_os_cfar_topic = "/arbe/feature/os_cfar";
string arbe_project_image_topic = "/arbe/feature/project_image/compressed";

unordered_map<pair<int, int>, vector<int>, PairHash>  point_map;