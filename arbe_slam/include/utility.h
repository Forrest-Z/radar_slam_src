#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#define PCL_NO_PRECOMPIL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>

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

// 使用自定义的点云类需要包括一下
// #define PCL_NO_PRECOMPIL
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/kdtree/impl/kdtree_flann.hpp>


struct EIGEN_ALIGN16 ArbeObjec
{
  PCL_ADD_POINT4D;
  float range;
  float azimuth;
  float elevation;
  float doppler;
  float power;
  float timestamp_ms;

  float length;
  float width;
  float high;
  int quantity;
  int label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(ArbeObjec,           // here we assume a XYZRGB + "detectionData" (as fields)
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, range, range)
  (float, azimuth, azimuth)
  (float, elevation, elevation)
  (float, doppler, doppler)
  (float, power, power)
  (float, timestamp_ms, timestamp_ms)

  (float, length, length)
  (float, width, width)
  (float, high, high)
  (int, quantity, quantity)
  (int, label, label)

)
typedef ArbeObjec PointAO;
typedef pcl::PointCloud<PointAO> PointCloudAO;

string arbe_origin_topic = "/arbe/rviz/pointcloud";
string arbe_segment_topic = "/arbe/rviz/segment";
string arbe_outlier_topic = "/arbe/rviz/outlier";
string arbe_static_topic = "/arbe/rviz/static";
string arbe_moving_topic = "/arbe/rviz/moving";
string arbe_object_topic = "/arbe/rviz/object";

string arbe_ego_doppler_topic = "/arbe/ego_doppler";

string frame_id = "image_radar";

float range_res = 0.639;
float range_bias = -0.639;
float azimuth_res = -0.0156;
float azimuth_bias = 1.0014;
//-63.94890472226788 -0.8972650848906567 57.37911942429021
float elevation_res = -0.0212;
float elevation_bias = 0.339;
//-16.017821758490438 -1.2133574193589876 19.435342872634198
float power_res = 0.125;
float power_bias = 116.06;

float MAX_SEGMENT_DISTANCE = 0.7;
float MIN_SEGMENT_DISTANCE = 0.2;
float MIN_SEGMENT_NUMBER = 3;
float MAX_SEGMENT_DOPPLER_THRE = 0.2;
int SEARCH_RAIUS_RATE = 30;
float STATIC_MOVING_THRE = 0.3;

int MAX_OBJECT = 128 * 5;
int OUTLIER_LABEL = 99999;

float MIN_CALCULATE_EGO_DOPPLER_RADIUS = 30;

int range_number = 640;
int azimuth_number = 128;
int elevation_number = 32;


double motion_estimate[3] = { 0,0 };

