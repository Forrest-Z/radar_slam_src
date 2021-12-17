#include <ros/ros.h>
#include <rosbag/bag.h>
#include <string>
#include <cstdlib>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

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


void insert_noise(PointCloudA::Ptr pointcloud_pcl)
{
  int azimuth_noise_index = rand() % 128;
  double azimuth_noise = azimuth_noise_index * azimuth_res + azimuth_bias;

  int noise_number = rand() % 800 + 200;
  for (int i = 0;i < noise_number;i++)
  {
    pointcloud_pcl->push_back(get_noise_point(azimuth_noise));
  }
}

PointA get_noise_point(double azimuth_noise)
{
  PointA point;


}

int main(int argc, char** argv)
{
  string bag_file = "media/qinguoyu/ubuntu_data/bag/arbe/wrs42/college_town_2/wrs42_college_02_10.bag";
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Write);

  vector<string> topics;
  topics.push_back(std::string("/arbe/rviz/pointcloud"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::PointCloud2ConstPtr pointcloud_ros = message.instantiate<sensor_msgs::PointCloud2>();
    if (pointcloud_ros != NULL)
    {
      PointCloudA::Ptr pointcloud_pcl;
      pcl::fromROSMsg(*pointcloud_ros, *pointcloud_pcl);
      insert_noise(pointcloud_pcl);
    }


  }
  bag.close();
}

