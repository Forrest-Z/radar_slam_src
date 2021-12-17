#include "utility.h"



class RadarMapping
{
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_feature_sub;
  ros::Subscriber arbe_origin_sub;
  ros::Subscriber arbe_radar_odometry_sub;


  ros::Publisher arbe_mapping_odometry_pub;
  ros::Publisher arbe_mapping_path_pub;
  ros::Publisher arbe_mapping_pub;
  ros::Publisher arbe_feature_mapped_pub;
  ros::Publisher arbe_submap_pub;
  ros::Publisher  icp_score_pub;
  tf::TransformBroadcaster map_broadcaster;

  PointCloudA::Ptr arbe_origin_pcl;
  PointCloudA::Ptr arbe_feature_old_pcl;
  PointCloudA::Ptr arbe_feature_old_pcl;
  PointCloudA::Ptr arbe_feature_buf_pcl;
  PointCloudA::Ptr arbe_feature_calc_pcl;
  PointCloudA::Ptr arbe_feature_old_buf_pcl;
  PointCloudA::Ptr arbe_after_mapping_pcl;

  PointCloudI::Ptr arbe_feature_xyzi_pcl;
  PointCloudI::Ptr arbe_feature_xyzi_rt_pcl;
  PointCloudI::Ptr arbe_feature_match_pcl;
  PointCloudI::Ptr arbe_feature_match_old_pcl;
  PointCloudI::Ptr arbe_feature_mapped_pcl;


  mutex mBuf;

  Eigen::Quaterniond q_base_map;
  Eigen::Vector3d t_base_map;

  Eigen::Quaterniond q_odo_base;
  Eigen::Vector3d t_odo_base;

  Eigen::Quaterniond q_odo_map;
  Eigen::Vector3d t_odo_map;

  Eigen::Quaterniond q_odo_map_calc;
  Eigen::Vector3d t_odo_map_calc;

  Eigen::Quaterniond q_odo_map_old;
  Eigen::Vector3d t_odo_map_old;

  Eigen::Quaterniond q_odo_map_opt;
  Eigen::Vector3d t_odo_map_opt;

  double arbe_origin_time;
  double arbe_feature_time;
  double arbe_radar_odometry_time;

  bool  arbe_feature_new;
  bool  arbe_origin_new;
  bool  arbe_radar_odometry_new;


  PointI pre_pose;
  PointI curr_pose;

  PointCloudI::Ptr key_pose_t;
  PointCloudQT::Ptr key_pose_qt;
  deque<PointCloudI::Ptr> key_frame_pcl;

  PointCloudI::Ptr key_pose_surround_t;
  PointCloudI::Ptr key_pose_surround_us_t;
  PointCloudI::Ptr key_pose_surround_exist_t;

  deque<PointCloudI::Ptr> key_frame_surround_exist_list;
  vector<int> key_frame_surround_index_list;

  pcl::KdTreeFLANN<PointI>::Ptr kdtree_key_frame;
  std::vector<int> point_search_ind;
  std::vector<float> point_search_dist;


  PointCloudI::Ptr submap_pcl;
  PointCloudI::Ptr submap_us_pcl;

  pcl::UniformSampling<PointI> key_pose_us;
  pcl::UniformSampling<PointI> sub_map_us;
  pcl::UniformSampling<PointI> key_frame_us;

  pcl::IterativeClosestPoint<PointI, PointI> icp;

  double yaw_last;
  double yaw_curr;

  nav_msgs::Path radar_path;

  std_msgs::Header cloud_header;
  std_msgs::Header sub_cloud_header;

  Eigen::Quaterniond q_odo_map_buf;
  Eigen::Vector3d t_odo_map_buf;

  Eigen::Quaterniond q_odo_map_old_buf;
  Eigen::Vector3d t_odo_map_old_buf;
public:
  RadarMapping() :nh("~")
  {
    arbe_feature_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_static_front_topic, 1,
      &RadarMapping::arbe_feature_cb, this);
    arbe_origin_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_origin_topic, 1,
      &RadarMapping::arbe_origin_cb, this);
    arbe_radar_odometry_sub = nh.subscribe<nav_msgs::Odometry>(arbe_radar_odometry_topic, 1,
      &RadarMapping::arbe_radar_odometry_cb, this);

    arbe_mapping_odometry_pub = nh.advertise<nav_msgs::Odometry>(arbe_mapping_odometry_topic, 100);
    arbe_mapping_path_pub = nh.advertise<nav_msgs::Path>(arbe_mapping_path_topic, 100);
    // arbe_mapping_pub = nh.advertise<sensor_msgs::PointCloud2 >(arbe_mapping_topic, 10);
    arbe_feature_mapped_pub = nh.advertise<sensor_msgs::PointCloud2 >(arbe_feature_mapped_topic, 10);
    arbe_submap_pub = nh.advertise<sensor_msgs::PointCloud2 >(arbe_submap_topic, 10);
    icp_score_pub = nh.advertise < std_msgs::Float64>(mapping_icp_score_topic, 1);

    q_base_map = Eigen::Quaterniond(1, 0, 0, 0);
    t_base_map = Eigen::Vector3d(0, 0, 0);

    q_odo_base = Eigen::Quaterniond(1, 0, 0, 0);
    t_odo_base = Eigen::Vector3d(0, 0, 0);

    q_odo_map = Eigen::Quaterniond(1, 0, 0, 0);
    t_odo_map = Eigen::Vector3d(0, 0, 0);

    q_odo_map_buf = Eigen::Quaterniond(1, 0, 0, 0);
    t_odo_map_buf = Eigen::Vector3d(0, 0, 0);

    arbe_origin_time = 0;
    arbe_feature_time = 0;
    arbe_radar_odometry_time = 0;

    arbe_feature_new = false;
    arbe_origin_new = false;
    arbe_radar_odometry_new = false;

    key_frame_us.setRadiusSearch(0.1);
    key_pose_us.setRadiusSearch(1.0);
    sub_map_us.setRadiusSearch(0.2);

    curr_pose.x = pre_pose.x = 0;
    curr_pose.y = pre_pose.y = 0;
    curr_pose.z = pre_pose.z = 0;
    curr_pose.intensity = pre_pose.intensity = 0;

    icp.setMaxCorrespondenceDistance(1.5);
    icp.setMaximumIterations(10000);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setMaximumIterations(MAX_ICP_NUMBER);

    yaw_last = yaw_curr = 0;

    radar_path.header.frame_id = "base_link";

    allocate_memory();

  }

  void allocate_memory()
  {
    arbe_origin_pcl.reset(new PointCloudA());
    arbe_feature_old_pcl.reset(new PointCloudA());
    arbe_feature_buf_pcl.reset(new PointCloudA());
    arbe_feature_calc_pcl.reset(new PointCloudA());
    arbe_feature_old_buf_pcl.reset(new PointCloudA());

    arbe_feature_xyzi_pcl.reset(new PointCloudI());
    arbe_feature_xyzi_rt_pcl.reset(new PointCloudI());
    arbe_feature_match_pcl.reset(new PointCloudI());
    arbe_feature_match_old_pcl.reset(new PointCloudI());
    arbe_feature_mapped_pcl.reset(new PointCloudI());
    arbe_after_mapping_pcl.reset(new PointCloudA());

    key_pose_t.reset(new PointCloudI());
    key_pose_qt.reset(new PointCloudQT());

    key_pose_surround_t.reset(new PointCloudI());
    key_pose_surround_us_t.reset(new PointCloudI());
    key_pose_surround_exist_t.reset(new PointCloudI());

    submap_pcl.reset(new PointCloudI());
    submap_us_pcl.reset(new PointCloudI());

    kdtree_key_frame.reset(new pcl::KdTreeFLANN<PointI>());
  }

  void arbe_origin_cb(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {
    sub_cloud_header = arbe_origin_ros->header;
    arbe_origin_time = arbe_origin_ros->header.stamp.toSec();
    arbe_origin_pcl->clear();
    pcl::fromROSMsg(*arbe_origin_ros, *arbe_origin_pcl);
    arbe_origin_new = true;
  }

  void arbe_feature_cb(const sensor_msgs::PointCloud2ConstPtr& arbe_feature_ros)
  {
    arbe_feature_time = arbe_feature_ros->header.stamp.toSec();
    arbe_feature_buf_pcl->clear();
    pcl::fromROSMsg(*arbe_feature_ros, *arbe_feature_buf_pcl);
    arbe_feature_new = true;
  }

  void arbe_radar_odometry_cb(const nav_msgs::Odometry::ConstPtr& arbe_radar_odometry_ros)
  {
    arbe_radar_odometry_time = arbe_radar_odometry_ros->header.stamp.toSec();
    // high frequence publish

    q_odo_base.x() = arbe_radar_odometry_ros->pose.pose.orientation.x;
    q_odo_base.y() = arbe_radar_odometry_ros->pose.pose.orientation.y;
    q_odo_base.z() = arbe_radar_odometry_ros->pose.pose.orientation.z;
    q_odo_base.w() = arbe_radar_odometry_ros->pose.pose.orientation.w;
    t_odo_base.x() = arbe_radar_odometry_ros->pose.pose.position.x;
    t_odo_base.y() = arbe_radar_odometry_ros->pose.pose.position.y;
    t_odo_base.z() = arbe_radar_odometry_ros->pose.pose.position.z;

    arbe_radar_odometry_new = true;

    q_odo_map_old_buf = q_odo_map_buf;
    t_odo_map_old_buf = t_odo_map_buf;

    q_odo_map_buf = q_base_map * q_odo_base;
    t_odo_map_buf = q_base_map * t_odo_base + t_base_map;

    nav_msgs::Odometry arbe_mapping_odometry_ros;
    arbe_mapping_odometry_ros.header = arbe_radar_odometry_ros->header;
    arbe_mapping_odometry_ros.child_frame_id == "mapping";
    arbe_mapping_odometry_ros.pose.pose.orientation.x = q_odo_map_buf.x();
    arbe_mapping_odometry_ros.pose.pose.orientation.y = q_odo_map_buf.y();
    arbe_mapping_odometry_ros.pose.pose.orientation.z = q_odo_map_buf.z();
    arbe_mapping_odometry_ros.pose.pose.orientation.w = q_odo_map_buf.w();
    arbe_mapping_odometry_ros.pose.pose.position.x = t_odo_map_buf.x();
    arbe_mapping_odometry_ros.pose.pose.position.y = t_odo_map_buf.y();
    arbe_mapping_odometry_ros.pose.pose.position.z = t_odo_map_buf.z();
    arbe_mapping_odometry_pub.publish(arbe_mapping_odometry_ros);

    // path
    geometry_msgs::PoseStamped radar_pose_ros;
    radar_pose_ros.header = arbe_radar_odometry_ros->header;
    radar_pose_ros.pose = arbe_mapping_odometry_ros.pose.pose;
    radar_path.poses.push_back(radar_pose_ros);
    arbe_mapping_path_pub.publish(radar_path);

    // tf
    geometry_msgs::TransformStamped odom_trans_ros;
    odom_trans_ros.header = arbe_radar_odometry_ros->header;
    odom_trans_ros.child_frame_id = "mapping";
    odom_trans_ros.transform.translation.x = t_odo_base.x();
    odom_trans_ros.transform.translation.y = t_odo_base.y();
    odom_trans_ros.transform.translation.z = t_odo_base.z();
    odom_trans_ros.transform.rotation = arbe_radar_odometry_ros->pose.pose.orientation;
    map_broadcaster.sendTransform(odom_trans_ros);


  }

  void run()
  {
    if (arbe_feature_new && arbe_feature_new && std::abs(arbe_feature_time - arbe_radar_odometry_time) < 0.005)
    {
      clock_t start = clock();
      save_data();

      pre_process_feature();

      scan_to_scan();

      clock_t end = clock();
      double duration = (double)(end - start) / CLOCKS_PER_SEC;
      ROS_INFO("RadarMapping duration : %f", duration);
    };
  }


  void save_data()
  {
    mBuf.lock();

    q_odo_map_calc = q_odo_map_buf;
    t_odo_map_calc = t_odo_map_buf;

    q_odo_map_old = q_odo_map_old_buf;
    t_odo_map_old = t_odo_map_old_buf;

    Eigen::Vector3d e_odo_map_calc = q_odo_map_calc.matrix().eulerAngles(2, 1, 0);
    yaw_curr = e_odo_map_calc(2);

    cloud_header = sub_cloud_header;

    *arbe_feature_calc_pcl = *arbe_feature_buf_pcl;
    *arbe_feature_old_pcl = *arbe_feature_old_buf_pcl;

    arbe_feature_new = false;
    mBuf.unlock();
  }


  void pre_process_feature()
  {
    pcl::copyPointCloud(*arbe_feature_calc_pcl, *arbe_feature_xyzi_pcl);

    for (int i = 0; i < arbe_feature_xyzi_pcl->size();i++)
    {
      arbe_feature_xyzi_pcl->points[i].z = 0;
    }

    Eigen::Isometry3d  odometry_rt(q_odo_map_calc);
    odometry_rt.pretranslate(t_odo_map_calc);
    pcl::transformPointCloud(*arbe_feature_xyzi_pcl, *arbe_feature_xyzi_rt_pcl, odometry_rt.matrix());


    key_frame_us.setInputCloud(arbe_feature_xyzi_rt_pcl);
    key_frame_us.filter(*arbe_feature_match_pcl);


    pcl::copyPointCloud(*arbe_feature_old_pcl, *arbe_feature_xyzi_pcl);

    for (int i = 0; i < arbe_feature_xyzi_pcl->size();i++)
    {
      arbe_feature_xyzi_pcl->points[i].z = 0;
    }

    Eigen::Isometry3d  odometry_rt(q_odo_map_calc);
    odometry_rt.pretranslate(t_odo_map_calc);
    pcl::transformPointCloud(*arbe_feature_xyzi_pcl, *arbe_feature_xyzi_rt_pcl, odometry_rt.matrix());


    key_frame_us.setInputCloud(arbe_feature_xyzi_rt_pcl);
    key_frame_us.filter(*arbe_feature_match_pcl);


    ROS_INFO("copy_pointcloud : get %d feature point, us get %d us point",
      (int)arbe_feature_buf_pcl->size(), (int)arbe_feature_match_pcl->size());
  }


  void publish_sub_map()
  {
    // sensor_msgs::PointCloud2 arbe_submap_ros;
    pcl::toROSMsg(*submap_us_pcl, arbe_submap_ros);
    arbe_submap_ros.header = cloud_header;
    arbe_submap_pub.publish(arbe_submap_ros);

    sensor_msgs::PointCloud2 arbe_feature_mapped_ros;
    pcl::toROSMsg(*arbe_feature_mapped_pcl, arbe_feature_mapped_ros);
    arbe_feature_mapped_ros.header = cloud_header;
    arbe_feature_mapped_pub.publish(arbe_feature_mapped_ros);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "radar_mapping");

  RadarMapping rm;

  // std::thread loopthread(&RadarMapping::loopClosureThread, &rm);

  ros::Rate rate(200);
  while (ros::ok())
  {
    ros::spinOnce();

    rm.run();

    rate.sleep();
  }
}