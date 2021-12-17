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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <queue>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <mutex>

#include "ceres_factor.hpp"
#include "arbe_slam/CANROS.h"
#include "ekf.h"

using namespace std;


//arbe
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
  int counter;
  float noise_range;
  float noise_azimuth;

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
  (int, counter, counter)
  (float, noise_range, noise_range)
  (float, noise_azimuth, noise_azimuth)
)

typedef ArbeObjec PointAO;
typedef pcl::PointCloud<PointAO> PointCloudAO;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointCloudI;


struct PointQT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  double qx;
  double qy;
  double qz;
  double qw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointQT,
  (double, x, x) (double, y, y)
  (double, z, z) (double, intensity, intensity)
  (double, qx, qx) (double, qy, qy) (double, qz, qz)(double, qw, qw)
  (double, time, time)
)
typedef pcl::PointCloud<PointQT> PointCloudQT;

struct Vector2iHash {
  size_t operator()(const vector<int>& v) const {
    std::hash<int> hasher;
    size_t seed = 0;

    seed ^= hasher(v[0]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hasher(v[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};

struct PairHash {
  size_t operator()(const pair<int, int>& p) const {
    std::hash<int> hasher;
    size_t seed = 0;

    seed ^= hasher(p.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hasher(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};

struct IntHash {
  size_t operator()(const int& i) const {
    std::hash<int> hasher;
    size_t seed = 0;

    seed ^= hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};


typedef struct
{
  int16_t angle;
  int16_t cmd;
  uint16_t speed;
  int8_t torque;
}SteeringReport;

typedef struct
{
  int max_power;
  vector<int> index;
  int max_index;
  int min_index;

}ProjectInfo;

float range_res = 0.639;
float range_bias = -0.639;
float azimuth_res = -0.0156;
float azimuth_bias = 1.0014;
//-63.94890472226788 -0.8972650848906567 57.37911942429021
float elevation_res = -0.0212;
float elevation_bias = 0.339;
//-16.017821758490438 -1.2133574193589876 19.435342872634198
float power_res = 0.;
float power_bias = 116.0;

//feature

string arbe_origin_topic = "/arbe/rviz/pointcloud";
string arbe_project_topic = "/arbe/feature/project";
string arbe_outlier_topic = "/arbe/feature/outlier";
string arbe_static_topic = "/arbe/feature/static";
string arbe_static_front_topic = "/arbe/feature/static_front";
string arbe_moving_topic = "/arbe/feature/moving";
string arbe_moving_front_topic = "/arbe/feature/moving_front";
string arbe_object_topic = "/arbe/feature/object";
string arbe_object_real_topic = "/arbe/feature/object_real";
string arbe_object_marker_topic = "/arbe/feature/object_marker";



int RANGE_WIDTH = 512;
int AZIMUTH_WIDTH = 128;
int ELEVATION_WIDTH = 32;

float MAX_SEGMENT_DISTANCE = 0.7;
float MIN_SEGMENT_DISTANCE = 0.2;
float MIN_SEGMENT_NUMBER = 4;
float MAX_SEGMENT_DOPPLER_THRE = 0.5;
int SEARCH_RAIUS_RATE = 30;
float STATIC_MOVING_THRE = 0.3;
float MAX_OBJECT_DISTANCE = 1;
float Min_OBJECT_DOPPLER = 0.2;
float MAX_OBJECT_DOPPLER = 1;

int MAX_OBJECT = 128 * 5;
int OUTLIER_LABEL = 99999;

float MIN_CALCULATE_EGO_DOPPLER_RADIUS = 20;


int range_number = 640;
int azimuth_number = 128;
int elevation_number = 32;

vector<vector<int>> neighbor_iterator_static =
{ {-1,0,0},{1,0,0},
{0,-1,0},{0,1,0},
{0,0,-1},{0,0,1} ,
{0,0,-2},{0,0,2},
{0,0,-3},{0,0,3},
{0,0,-4},{0,0,4} };

vector<vector<int>> neighbor_iterator_moving =
{ {-1,0,0},{1,0,0},
{0,-1,0},{0,1,0},
{0,0,-1},{0,0,1} ,
{-1,0,-1},{-1,0,1} ,
{1,0,-1},{1,0,1} };

vector<vector<int>> neighbor_iterator =
{ {-1,0,0},{1,0,0},
{0,-1,0},{0,1,0},
{0,0,-1},{0,0,1} };

double motion_estimate[2] = { 0,0 };

//需要使用xyz类型的点云
int MAX_ICP_NUMBER = 20;
float UNIFORM_SAMPLEING_RADIUS = 0.01;


//os-cfar
string arbe_os_cfar_topic = "/arbe/feature/os_cfar";
string arbe_project_image_topic = "/arbe/feature/project_image/compressed";
string dbw_topic = "/dbw_rx";

string arbe_ego_doppler_topic = "/arbe/feature/ego_doppler";
string arbe_ego_doppler_kf_topic = "/arbe/feature/ego_doppler_kf";
string arbe_ego_doppler_x_topic = "/arbe/feature/ego_doppler_x";
string arbe_ego_doppler_y_topic = "/arbe/feature/ego_doppler_y";

int OS_CFAR_RADIUS = 4;
int OS_CFAR_RADIUS_RANGE = 8;
int OS_CFAR_RADIUS_AZIMUTH = 4;
int MIN_OS_CFAR_NUMBER = 11;
int MAX_OS_CFAR_NUMBER = 22;
float OS_CFAR_A = 2;
int MAX_ELEVATION_NUMBER = 10;
int MAX_ELEVATION_RANGE = 24;
int ROW = 512;
int COL = 128;


int PITCH_BIAS = 0.1;
int Z_BIAS = 1.5;
int Z_THRE = 1;

//odometry
string arbe_radar_odometry_topic = "/arbe/odometry/radar_odometry";
string arbe_radar_path_topic = "/arbe/odometry/radar_path";
string arbe_feature_xyz_us_old_icp_topic = "/arbe/odometry/feature_xyz_us_old_icp";
string arbe_after_odometry_topic = "/arbe/odometry/arbe_after_odometry";
string icp_score_topic = "/arbe/odometry/icp_score";

float ODOMETRY_ICP_MCD = 3;

//mapping
string arbe_mapping_odometry_topic = "/arbe/mapping/radar_odometry";
string arbe_mapping_path_topic = "/arbe/mapping/radar_path";
string arbe_mapping_topic = "/arbe/mapping/mapping";
string arbe_feature_mapped_topic = "/arbe/mapping/feature_mapped";

string arbe_submap_topic = "/arbe/mapping/submap";
string mapping_icp_score_topic = "/arbe/mapping/icp_score";
string frame_id = "image_radar";


float MAPPING_ICP_MCD = 10;
float KEY_POSE_SEARCH_RADIUS = 25.0;
float  KEY_POSE_US_RADIUS = 0.01;
float  SUB_MAP_US_RADIUS = 0.01;

float KEY_POSE_T_THRESHOULD = 0.3;
float KEY_POSE_Q_THRESHOULD = 20 / 180. * 3.14159;

//tracing

string arbe_object_trace_topic = "/arbe/tracing/object_trace";
string arbe_object_predict_topic = "/arbe/tracing/object_predict";
string arbe_object_trace_valid_topic = "/arbe/tracing/object_trace_valid";

double MIN_COORELATION = 4;