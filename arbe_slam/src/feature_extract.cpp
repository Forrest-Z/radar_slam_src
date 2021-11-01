#define PCL_NO_PRECOMPIL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

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

// 使用自定义的点云类需要包括一下
// #define PCL_NO_PRECOMPIL
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/kdtree/impl/kdtree_flann.hpp>

string arbe_pc_topic = "/arbe/rviz/pointcloud";

float range_res = 0.639;
float range_bias = -0.639;
float azimuth_rest = -0.0156;
float azimuth_bias = 1.0014;
//-63.94890472226788 -0.8972650848906567 57.37911942429021
float elevation_res = -0.0212;
float elevation_bias = 0.339;
//-16.017821758490438 -1.2133574193589876 19.435342872634198
float power_res = 0.125;
float power_bias = 116.06;


float MIN_SEGMENT_DISTANCE = 0.7;
float MIN_SEGMENT_NUMBER = 3;
float MAX_SEGMENT_DOPPLER_DIFF = 0.2;

class FeatureExtrace {
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_pc_sub;
  PointCloudA::Ptr arbe_pc_pcl;

  pcl::KdTreeFLANN<PointA>::Ptr kdtree_origin;


  vector<int> point_label;
  vector<int> point_bfs_queue;

  int labelCount;


public:
  FeatureExtrace() :nh("~") {
    arbe_pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_pc_topic, 1, &FeatureExtrace::pointcloud_callback, this);
    allocateMemory();
  }

  void allocateMemory() {
    arbe_pc_pcl.reset(new PointCloudA());
    kdtree_origin.reset(new pcl::KdTreeFLANN<PointA>());

    point_label = new uint16_t[arbe_pc_pcl->size()];
    point_bfs_queue = new uint16_t[arbe_pc_pcl->size()];

  }

  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& arbe_pc_ros)
  {

    pcl::fromROSMsg(*arbe_pc_ros, *arbe_pc_pcl);

    kdtree_origin->setInputCloud(arbe_pc_pcl);
    labelCount = 1;

    for (size_t i = 0; i < arbe_pc_pcl->size();i++)
    {
      if (point_label[i] == 0)
      {
        label_pointcloud(i);
      }

    }
  }


  void label_pointcloud(int index) {

    point_bfs_queue.push_back(index);

    vector<int> segmentation_index();
    segmentation_index.push_back(index);

    while (point_bfs_queue.size() > 0)
    {
      PointA point_center = arbe_pc_pcl->points[point_bfs_queue.pop()];
      if (kdtree_origin->radiusSearch(point_center, MIN_SEGMENT_DISTANCE, point_search_ind, point_search_sq_dis) > 0)
      {
        for (int j = 0;j < point_search_ind.size();j++)
        {
          int index = point_search_ind[j];
          float distance = point_search_sq_dis[j];
          if (point_label[i] == 0)
          {
            point_search = arbe_pc_pcl->points[j];
            float doppler_diff = abs(point_center.doppler - point_search.doppler);
            if (doppler_diff < MAX_SEGMENT_DOPPLER_DIFF)
            {
              segmentation_index.push_back(j);
              point_bfs_queue.push_back(j);
            }
          }
        }
      }
      else(
        point_label[i] = -1;
      )

    }




  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_extrace");

  FeatureExtrace fe;

  ros::spin();
}


