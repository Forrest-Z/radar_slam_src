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
#include <queue>
#include <unordered_map>
#include <algorithm>

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
  (float, quantity, quantity)
  (float, label, label)

)
typedef ArbeObjec PointAO;
typedef pcl::PointCloud<PointAO> PointCloudAO;

string arbe_origin_topic = "/arbe/rviz/pointcloud";
string arbe_segment_topic = "/arbe/rviz/segment";
string arbe_outlier_topic = "/arbe/rviz/outlier";
string arbe_static_topic = "/arbe/rviz/static";
string arbe_moving_topic = "/arbe/rviz/moving";
string arbe_object_topic = "/arbe/rviz/object";
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
float STATIC_MOVING_THRE = 0.5;

int MAX_OBJECT = 128 * 5;
int OUTLIER_LABEL = 99999;


int range_number = 640;
int azimuth_number = 128;
int elevation_number = 32;


class FeatureExtrace {
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_origin_sub;
  ros::Publisher arbe_segment_pub;
  ros::Publisher arbe_outlier_pub;
  ros::Publisher arbe_static_pub;
  ros::Publisher arbe_moving_pub;
  ros::Publisher arbe_object_pub;

  PointCloudA::Ptr arbe_origin_pcl;
  PointCloudA::Ptr arbe_segment_pcl;
  PointCloudA::Ptr arbe_outlier_pcl;
  PointCloudA::Ptr arbe_static_pcl;
  PointCloudA::Ptr arbe_moving_pcl;
  PointCloudAO::Ptr arbe_object_pcl;


  std_msgs::Header cloud_header;
  pcl::KdTreeFLANN<PointA>::Ptr kdtree_origin;
  pcl::KdTreeFLANN<PointA>::Ptr kdtree_object;

  vector<int> point_label;
  vector<vector<int>> label_color;

  int label_count;

  vector<int> relate_doppler_list;
  vector<int> static_index_list;
  vector<int> moving_index_list;
  float ego_doppler;


public:
  FeatureExtrace() :nh("~") {
    arbe_origin_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_origin_topic, 1,
      &FeatureExtrace::pointcloud_callback, this);
    arbe_segment_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_segment_topic, 1);
    arbe_outlier_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_outlier_topic, 1);
    arbe_static_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_static_topic, 1);
    arbe_moving_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_moving_topic, 1);
    arbe_object_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_object_topic, 1);

    nh.getParam("max_segment_distance", MAX_SEGMENT_DISTANCE);
    nh.getParam("min_segment_distance", MIN_SEGMENT_DISTANCE);
    nh.getParam("min_segment_number", MIN_SEGMENT_NUMBER);
    nh.getParam("max_segment_doppler_diff", MAX_SEGMENT_DOPPLER_THRE);
    nh.getParam("search_radius_rate", SEARCH_RAIUS_RATE);
    nh.getParam("static_moving_thre", STATIC_MOVING_THRE);

    allocateMemory();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))//Info))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
    cout << "init!" << endl;
  }

  void allocateMemory() {
    arbe_origin_pcl.reset(new PointCloudA());
    arbe_segment_pcl.reset(new PointCloudA());
    arbe_outlier_pcl.reset(new PointCloudA());
    arbe_static_pcl.reset(new PointCloudA());
    arbe_moving_pcl.reset(new PointCloudA());
    arbe_object_pcl.reset(new PointCloudAO());

    kdtree_origin.reset(new pcl::KdTreeFLANN<PointA>());

    for (int i = 0;i < MAX_OBJECT;i++)
    {
      label_color.push_back(FeatureExtrace::get_lable_color(i));
    }
  }

  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {
    clock_t start, end_copy, end_doppler, end_segment, end_publish;
    start = clock();

    copy_pointcloud(arbe_origin_ros);
    end_copy = clock();

    calculate_ego_doppler();
    end_doppler = clock();

    segment_pointcloud();
    end_segment = clock();

    publish_pointcloud();
    end_publish = clock();

    double duration_copy = (double)(end_copy - start) / CLOCKS_PER_SEC;
    double duration_doppler = (double)(end_doppler - end_copy) / CLOCKS_PER_SEC;
    double duration_segment = (double)(end_segment - end_doppler) / CLOCKS_PER_SEC;
    double duration_publish = (double)(end_publish - end_segment) / CLOCKS_PER_SEC;

    double duration_total = duration_copy + duration_doppler + duration_segment + duration_publish;

    ROS_INFO("duration: %f + %f + %f + %f = %f.",
      duration_copy, duration_doppler, duration_segment, duration_publish, duration_total);
  }

  void copy_pointcloud(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {
    ROS_INFO("copy_pointcloud : get %d origin ROS point!", arbe_origin_ros->width);
    cloud_header = arbe_origin_ros->header;
    pcl::fromROSMsg(*arbe_origin_ros, *arbe_origin_pcl);
    kdtree_origin->setInputCloud(arbe_origin_pcl);
  }


  void segment_pointcloud()
  {
    static_index_list.clear();
    for (size_t i = 0; i < arbe_origin_pcl->size();i++)
    {
      if (abs(relate_doppler_list[i] - ego_doppler) > STATIC_MOVING_THRE)
      {
        static_index_list.push_back(i);
      }
    }

    ROS_INFO("segment_pointcloud : origin moving points : %d!", (int)static_index_list.size());

    label_count = 1;
    point_label.clear();
    point_label.resize(arbe_origin_pcl->size(), 0);
    for (int index : static_index_list)
    {
      if (point_label[index] == 0)
      {
        label_pointcloud(index);
        point_label[index] = 1;
        ROS_DEBUG("segment_pointcloud : label point %d.", index);
      }
    }
  }


  float calculate_ego_doppler()
  {
    unordered_map<string, int> doppler_counter;

    relate_doppler_list.clear();

    for (size_t i = 0; i < arbe_origin_pcl->size();i++)
    {
      PointA point = arbe_origin_pcl->points[i];
      float relate_doppler = point.doppler / (cos(point.azimuth) * cos(point.elevation));
      int relate_doppler_ten = (int)(10 * relate_doppler);
      relate_doppler_list.push_back(relate_doppler);

      if (doppler_counter.find(to_string(relate_doppler_ten)) == doppler_counter.end())
      {
        doppler_counter.emplace(to_string(relate_doppler_ten), 1);
      }
      else {
        doppler_counter[to_string(relate_doppler_ten)] += 1;
      }
    }
    PointA point_0 = arbe_origin_pcl->points[0];
    float ego_doppler_ten = (int)(10 * point_0.doppler / (cos(point_0.azimuth) * cos(point_0.elevation)));
    int max_counter = 0;

    for (auto iter = doppler_counter.begin(); iter != doppler_counter.end(); ++iter)
    {
      if (iter->second > max_counter)
      {
        stringstream stream(iter->first);
        stream >> ego_doppler_ten;
        max_counter = iter->second;
      }
    }
    ego_doppler = ego_doppler_ten / 10.;
    ROS_INFO("calculate_ego_doppler : ego doppler : %f, max counter : %d, total counter : %d.",
      ego_doppler, max_counter, (int)arbe_origin_pcl->size());
    doppler_counter.clear();

  }


  void label_pointcloud(int index) {

    vector<int> point_label_tmp;
    point_label_tmp.resize(arbe_origin_pcl->size(), 0);

    queue<int> point_bfs_queue;
    point_bfs_queue.push(index);
    point_label_tmp[index] = 1;

    vector<int> segmentation_index;
    segmentation_index.push_back(index);

    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dis;

    while (point_bfs_queue.size() > 0)
    {
      int centrer_index = point_bfs_queue.front();
      PointA point_center = arbe_origin_pcl->points[centrer_index];
      point_bfs_queue.pop();

      // ROS_DEBUG("label point %d. point_bfs_queue.size() %d.",
      //   centrer_index, (int)point_bfs_queue.size());
      // ROS_DEBUG("point %d, search radius %f.", centrer_index, point_center.range / SEARCH_RAIUS_RATE);

      float search_radius = point_center.range / SEARCH_RAIUS_RATE;
      if (search_radius < range_res * 1.5)
      {
        search_radius = range_res * 1.5;
      }

      if (kdtree_origin->radiusSearch(point_center, search_radius,
        point_search_ind, point_search_sq_dis) > 0)
      {
        for (int j = 0;j < point_search_ind.size();j++)
        {
          int search_index = point_search_ind[j];
          float distance = point_search_sq_dis[j];
          if (point_label[search_index] == 0 && point_label_tmp[search_index] == 0)
          {

            PointA point_search = arbe_origin_pcl->points[search_index];
            float doppler_diff = abs(point_center.doppler - point_search.doppler);
            float azimuth_diff = abs(point_center.azimuth - point_search.azimuth);
            float elevation_diff = abs(point_center.elevation - point_search.elevation);

            if (doppler_diff < MAX_SEGMENT_DOPPLER_THRE && (azimuth_diff < abs(azimuth_res * 2.5) && elevation_diff < abs(elevation_res * 2.5)))
            {
              segmentation_index.push_back(search_index);
              point_bfs_queue.push(search_index);
              point_label_tmp[search_index] = 1;
            }
          }
        }
      }
    }

    int label = OUTLIER_LABEL;

    if (segmentation_index.size() >= MIN_SEGMENT_NUMBER)
    {
      label = label_count;
      label_count += 1;
      get_object_point(label, segmentation_index);
    }

    for (int i : segmentation_index)
    {
      point_label[i] = label;
    }
    ROS_DEBUG("abel_pointcloud : label object %d with %d points.", label, (int)segmentation_index.size());
  }

  void get_object_point(int label, vector<int>& segmentation_index)
  {
    PointCloudA::Ptr object_pcl;
    vector<float> x_list;
    vector<float> y_list;
    vector<float> z_list;
    vector<float> doppler_list;
    vector<float> power_list;

    for (int index : segmentation_index)
    {
      PointA point = arbe_origin_pcl->points[index];
      x_list.push_back(point.x);
      y_list.push_back(point.y);
      z_list.push_back(point.z);
      doppler_list.push_back(point.doppler);
      power_list.push_back(point.power);
    }

    float x_min = *min_element(x_list.begin(), x_list.end());
    float x_max = *max_element(x_list.begin(), x_list.end());

    float y_min = *min_element(y_list.begin(), y_list.end());
    float y_max = *max_element(y_list.begin(), y_list.end());

    float z_min = *min_element(z_list.begin(), z_list.end());
    float z_max = *max_element(z_list.begin(), z_list.end());

    float doppler_min = *min_element(doppler_list.begin(), doppler_list.end());
    float doppler_max = *max_element(doppler_list.begin(), doppler_list.end());

    float power_min = *min_element(power_list.begin(), power_list.end());
    float power_max = *max_element(power_list.begin(), power_list.end());


    float x_mean = (x_min + x_max) / 2;
    float width = (x_max - x_min);
    float y_mean = (y_min + y_max) / 2;
    float length = (y_max - y_min);
    float z_mean = (z_min + z_max) / 2;
    float high = (z_max - z_min);

    float doppler_mean = (doppler_min + doppler_max) / 2;
    float power_mean = (power_min + power_max) / 2;

    PointAO arbe_object;
    arbe_object.x = x_mean;
    arbe_object.y = y_mean;
    arbe_object.z = z_mean;

    arbe_object.doppler = doppler_mean;
    arbe_object.power = power_max;

    arbe_object.width = width;
    arbe_object.length = length;
    arbe_object.high = high;

    arbe_object_pcl->push_back(arbe_object);
  }

  void publish_pointcloud()
  {
    clock_t start, end_1, end_2, end_3, end_4, end_5, end_6, end_7;
    start = clock();

    arbe_segment_pcl->clear();
    end_1 = clock();
    arbe_outlier_pcl->clear();
    end_2 = clock();
    for (int i = 0;i < arbe_origin_pcl->size();i++)
    {
      int label = point_label[i];
      if (label == OUTLIER_LABEL)
      {
        arbe_outlier_pcl->push_back(arbe_origin_pcl->points[i]);
      }
      else if (label != 0)
      {
        arbe_segment_pcl->push_back(arbe_origin_pcl->points[i]);
        arbe_segment_pcl->back().power_value = label;
      }
      else {
        arbe_static_pcl->push_back(arbe_origin_pcl->points[i]);
      }
    }
    end_3 = clock();
    ROS_INFO("publish_pointcloud : segment %d segment point into %d objects, get %d outlier!",
      (int)arbe_segment_pcl->size(), (int)label_count, (int)arbe_outlier_pcl->size());

    sensor_msgs::PointCloud2 arbe_segment_ros;
    pcl::toROSMsg(*arbe_segment_pcl, arbe_segment_ros);
    arbe_segment_ros.header = cloud_header;
    arbe_segment_pub.publish(arbe_segment_ros);
    end_4 = clock();

    sensor_msgs::PointCloud2 arbe_outlier_ros;
    pcl::toROSMsg(*arbe_outlier_pcl, arbe_outlier_ros);
    arbe_outlier_ros.header = cloud_header;
    arbe_outlier_pub.publish(arbe_outlier_ros);
    end_5 = clock();

    sensor_msgs::PointCloud2 arbe_static_ros;
    pcl::toROSMsg(*arbe_static_pcl, arbe_static_ros);
    arbe_static_ros.header = cloud_header;
    arbe_static_pub.publish(arbe_static_ros);
    end_6 = clock();

    sensor_msgs::PointCloud2 arbe_object_ros;
    pcl::toROSMsg(*arbe_object_pcl, arbe_object_ros);
    arbe_object_ros.header = cloud_header;
    arbe_object_pub.publish(arbe_object_ros);
    arbe_object_pcl->clear();
    end_7 = clock();

    double duration_1 = (double)(end_1 - start) / CLOCKS_PER_SEC;
    double duration_2 = (double)(end_2 - start) / CLOCKS_PER_SEC;
    double duration_3 = (double)(end_3 - start) / CLOCKS_PER_SEC;
    double duration_4 = (double)(end_4 - start) / CLOCKS_PER_SEC;
    double duration_5 = (double)(end_5 - start) / CLOCKS_PER_SEC;
    double duration_6 = (double)(end_6 - start) / CLOCKS_PER_SEC;
    double duration_7 = (double)(end_7 - start) / CLOCKS_PER_SEC;


    ROS_DEBUG(" %f -> %f -> %f -> %f -> %f -> %f -> %f ",
      duration_1, duration_2, duration_3, duration_4, duration_5, duration_6, duration_7);

  }


  vector<int> get_lable_color(int index)
  {
    int group_size = 128;
    int group_num = 5;
    int group = (index < 640) ? (index / group_size) : 5;
    vector<int> ret;
    vector<vector<int>> color = { { 255, 0, 255 }, { 0, 0, 255 }, { 0, 255, 255 },
     { 0, 255, 0 }, { 255,255, 0 }, { 255, 255, 255 } };
    ret = color[group];
    switch (group)
    {
    case 0:
      ret[0] = (int)(255 - (index % group_size) * group_num);
      break;
    case 1:
      ret[1] = (int)((index % group_size) * group_num);
      break;
    case 2:
      ret[2] = (int)(255 - (index % group_size) * group_num);
      break;
    case 3:
      ret[0] = (int)((index % group_size) * group_num);
      break;
    case 4:
      ret[1] = (int)((255 - index % group_size) * group_num);
      break;
    default:
      break;
    }
    // cout << dopplor << "\t" << min_doppler << "\t" << dopplor_range << "\t" << index << "\t" << ret << endl;
    return ret;
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_extrace");

  FeatureExtrace fe;

  ros::spin();
}


