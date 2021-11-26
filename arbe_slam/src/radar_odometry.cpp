#include "utility.h"

class RadarOdometry
{
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_feature_sub;
  ros::Publisher arbe_radar_odometry_pub;
  ros::Publisher arbe_radar_path_pub;
  ros::Publisher arbe_feature_xyz_us_old_icp_pub;
  ros::Publisher arbe_after_odometry_pub;
  ros::Publisher icp_score_pub;
  tf::TransformBroadcaster odom_broadcaster;

  PointCloudA::Ptr arbe_feature_pcl;
  PointCloudA::Ptr arbe_feature_pcl_old;
  PointCloudA::Ptr arbe_after_odometry_pcl;

  PointCloudT::Ptr arbe_feature_xyz_pcl;
  PointCloudT::Ptr arbe_feature_xyz_old_pcl;
  PointCloudT::Ptr arbe_feature_xyz_old_icp_pcl;

  PointCloudT::Ptr arbe_feature_xyz_us_pcl;
  PointCloudT::Ptr arbe_feature_xyz_us_old_pcl;
  PointCloudT::Ptr arbe_feature_xyz_us_pre_pcl;
  PointCloudT::Ptr arbe_feature_xyz_us_old_icp_pcl;


  std_msgs::Header cloud_header;

  bool init;

  // string frame_if;
  // string child_frame_id;

  pcl::IterativeClosestPoint<PointT, PointT> icp;

  Eigen::Quaterniond q_odo_last;
  Eigen::Vector3f e_odo_last;
  Eigen::Vector3d t_odo_last;

  Eigen::Quaterniond q_odo_pre;
  Eigen::Vector3d t_odo_pre;

  Eigen::Quaterniond q_odo_base;
  Eigen::Vector3d t_odo_base;

  nav_msgs::Path radar_path;

  pcl::UniformSampling<PointT> US;

  Eigen::Matrix4f qt_odo_base;
  Eigen::Matrix4f qt_odo_last;
  Eigen::Matrix4f qt_odo_pre;
  Eigen::Matrix4f qt_pre_last;

public:
  RadarOdometry() :nh("~") {
    arbe_feature_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_static_front_topic, 1,
      &RadarOdometry::arbe_feature_cb, this);

    arbe_radar_odometry_pub = nh.advertise<nav_msgs::Odometry>(arbe_radar_odometry_topic, 100);
    arbe_radar_path_pub = nh.advertise<nav_msgs::Path>(arbe_radar_path_topic, 100);
    arbe_feature_xyz_us_old_icp_pub = nh.advertise<sensor_msgs::PointCloud2 >(arbe_feature_xyz_us_old_icp_topic, 10);
    arbe_after_odometry_pub = nh.advertise<sensor_msgs::PointCloud2 >(arbe_after_odometry_topic, 10);
    icp_score_pub = nh.advertise < std_msgs::Float64>(icp_score_topic, 1);

    init = false;
    nh.getParam("max_icp_number", MAX_ICP_NUMBER);
    nh.getParam("uniform_sampleing_radius", UNIFORM_SAMPLEING_RADIUS);

    icp.setMaxCorrespondenceDistance(3);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setMaximumIterations(MAX_ICP_NUMBER);

    US.setRadiusSearch(0.2f);//设置滤波时创建球体的半径

    q_odo_last = Eigen::Quaterniond(1, 0, 0, 0);
    e_odo_last = Eigen::Vector3f(0, 0, 0);
    t_odo_last = Eigen::Vector3d(0, 0, 0);

    q_odo_pre = Eigen::Quaterniond(1, 0, 0, 0);
    t_odo_pre = Eigen::Vector3d(0, 0, 0);

    q_odo_base = Eigen::Quaterniond(1, 0, 0, 0);
    t_odo_base = Eigen::Vector3d(0, 0, 0);

    radar_path.header.stamp = cloud_header.stamp;
    radar_path.header.frame_id = "base_link";

    qt_odo_base = Eigen::Matrix4f::Identity();
    qt_odo_pre = Eigen::Matrix4f::Identity();

    allocate_memory();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))//Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
    cout << "RadarOdometry init!" << endl;

  }

  void allocate_memory() {
    arbe_feature_pcl.reset(new PointCloudA());
    arbe_feature_pcl_old.reset(new PointCloudA());
    arbe_after_odometry_pcl.reset(new PointCloudA());

    arbe_feature_xyz_pcl.reset(new PointCloudT());
    arbe_feature_xyz_old_pcl.reset(new PointCloudT());
    arbe_feature_xyz_old_icp_pcl.reset(new PointCloudT());


    arbe_feature_xyz_us_pcl.reset(new PointCloudT());
    arbe_feature_xyz_us_old_pcl.reset(new PointCloudT());
    arbe_feature_xyz_us_pre_pcl.reset(new PointCloudT());
    arbe_feature_xyz_us_old_icp_pcl.reset(new PointCloudT());
  }

  void arbe_feature_cb(const sensor_msgs::PointCloud2ConstPtr& arbe_feature_ros)
  {
    clock_t start = clock();
    copy_pointcloud(arbe_feature_ros);
    uniform_sample_pointcloud();
    calculate_icp();
    publish_odometry();
    publish_pointcloud_icp();
    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("radar odometry duration : %f", duration);
    if (duration > 0.07)
    {
      ROS_WARN("RadarOdometry process take more than 70ms!");
    }
  }

  void copy_pointcloud(const sensor_msgs::PointCloud2ConstPtr& arbe_feature_ros)
  {
    cloud_header = arbe_feature_ros->header;
    pcl::fromROSMsg(*arbe_feature_ros, *arbe_feature_pcl);
    pcl::copyPointCloud(*arbe_feature_pcl, *arbe_feature_xyz_pcl);
    ROS_INFO("copy_pointcloud : get %d feature point!", (int)arbe_feature_ros->width);
  }

  void uniform_sample_pointcloud()
  {
    for (size_t i = 0;i < arbe_feature_xyz_pcl->size();i++)
    {
      arbe_feature_xyz_pcl->points[i].z = 0;
    }

    US.setInputCloud(arbe_feature_xyz_pcl);
    US.filter(*arbe_feature_xyz_us_pcl);

    ROS_INFO("uniform_sample_pointcloud : origin %d points, uniform sample %d points!",
      (int)arbe_feature_xyz_pcl->size(), (int)arbe_feature_xyz_us_pcl->size());
  }

  void calculate_icp()
  {
    if (init)
    {
      // Eigen::Isometry3d  odometry_rt_pre(q_odo_pre);
      // odometry_rt_pre.pretranslate(t_odo_pre);
      // pcl::transformPointCloud(*arbe_feature_xyz_us_pcl, *arbe_feature_xyz_us_pre_pcl, odometry_rt_pre.matrix());

      pcl::transformPointCloud(*arbe_feature_xyz_us_pcl, *arbe_feature_xyz_us_pre_pcl, qt_odo_pre);

      icp.setInputSource(arbe_feature_xyz_us_pre_pcl);
      icp.setInputTarget(arbe_feature_xyz_us_old_pcl);
      icp.align(*arbe_feature_xyz_us_old_icp_pcl);

      ROS_INFO("calculate_icp : MAX_ICP_NUMBER %d", MAX_ICP_NUMBER);

      ROS_INFO("has converged : %d score: %f", icp.hasConverged(), icp.getFitnessScore());
      // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
      //   icp.getFitnessScore() << std::endl;
      // std::cout << icp.getFinalTransformation() << std::endl;

      std_msgs::Float64 icp_score_msg;
      icp_score_msg.data = (float)icp.getFitnessScore();
      icp_score_pub.publish(icp_score_msg);

      qt_pre_last = icp.getFinalTransformation();
      qt_odo_last = qt_pre_last * qt_odo_pre;

      qt_odo_base = qt_odo_last * qt_odo_base;
      qt_odo_pre = qt_odo_last;

      // t_last_odo = qt_last_odo.topRightCorner(3, 1); 数据转换问题
      t_odo_last.x() = qt_odo_last(0, 3);
      t_odo_last.y() = qt_odo_last(1, 3);
      t_odo_last.z() = qt_odo_last(2, 3);
      e_odo_last = qt_odo_last.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

      Eigen::AngleAxisd rollAngle(e_odo_last(2), Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(e_odo_last(1), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(e_odo_last(0), Eigen::Vector3d::UnitZ());
      q_odo_last = yawAngle * pitchAngle * rollAngle;

      ROS_INFO("calculate_icp :yaw %f x %f y %f", e_odo_last(0), t_odo_last.x(), t_odo_last.y());

      cout << e_odo_last << endl;
      cout << t_odo_last << endl;

      t_odo_base = t_odo_base + q_odo_base * t_odo_last;
      q_odo_base = q_odo_base * q_odo_last;

      Eigen::Isometry3d  odometry_rt(q_odo_base);
      odometry_rt.pretranslate(t_odo_base);
      pcl::transformPointCloud(*arbe_feature_pcl, *arbe_after_odometry_pcl, odometry_rt.matrix());

    }
    else {
      init = true;
    }

    *arbe_feature_xyz_us_old_pcl = *arbe_feature_xyz_us_pcl;
  }


  void publish_odometry()
  {
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_odo_base.x(), q_odo_base.y(), q_odo_base.z(), q_odo_base.w())).getRPY(roll, pitch, yaw);

    // odometry
    std_msgs::Header header;
    header.stamp = cloud_header.stamp;
    header.frame_id = "base_link";

    nav_msgs::Odometry radar_odometry;
    radar_odometry.header = header;
    radar_odometry.child_frame_id = "odometry";
    radar_odometry.pose.pose.orientation.x = q_odo_base.x();
    radar_odometry.pose.pose.orientation.y = q_odo_base.y();
    radar_odometry.pose.pose.orientation.z = q_odo_base.z();
    radar_odometry.pose.pose.orientation.w = q_odo_base.w();
    radar_odometry.pose.pose.position.x = t_odo_base.x();
    radar_odometry.pose.pose.position.y = t_odo_base.y();
    radar_odometry.pose.pose.position.z = t_odo_base.z();
    // radar_odometry.twist.twist.angular.x = roll;
    // radar_odometry.twist.twist.angular.y = pitch;
    // radar_odometry.twist.twist.angular.z = yaw;
    // radar_odometry.twist.twist.linear.x = w_curr_t.x();
    // radar_odometry.twist.twist.linear.y = w_curr_t.y();
    // radar_odometry.twist.twist.linear.z = w_curr_t.z();
    arbe_radar_odometry_pub.publish(radar_odometry);

    // path
    geometry_msgs::PoseStamped radar_pose;
    radar_pose.header = header;
    radar_pose.pose = radar_odometry.pose.pose;

    radar_path.poses.push_back(radar_pose);
    arbe_radar_path_pub.publish(radar_path);

    // tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = header;
    odom_trans.child_frame_id = "odometry";
    odom_trans.transform.translation.x = t_odo_base.x();
    odom_trans.transform.translation.y = t_odo_base.y();
    odom_trans.transform.translation.z = t_odo_base.z();
    odom_trans.transform.rotation = radar_odometry.pose.pose.orientation;
    odom_broadcaster.sendTransform(odom_trans);
  }

  void publish_pointcloud_icp()
  {
    sensor_msgs::PointCloud2 arbe_feature_xyz_us_old_icp_ros;
    pcl::toROSMsg(*arbe_feature_xyz_us_old_icp_pcl, arbe_feature_xyz_us_old_icp_ros);
    arbe_feature_xyz_us_old_icp_ros.header = cloud_header;
    arbe_feature_xyz_us_old_icp_pub.publish(arbe_feature_xyz_us_old_icp_ros);
    arbe_feature_xyz_us_old_icp_pcl.reset(new PointCloudT());

    sensor_msgs::PointCloud2 arbe_after_odometry_ros;
    pcl::toROSMsg(*arbe_after_odometry_pcl, arbe_after_odometry_ros);
    arbe_after_odometry_ros.header = cloud_header;
    arbe_after_odometry_pub.publish(arbe_after_odometry_ros);
    arbe_after_odometry_pcl.reset(new PointCloudA());

  }

  void save_pcl(PointCloudA::Ptr target_pcl)
  {
    ostringstream oss1;
    oss1 << cloud_header.stamp.sec;
    ostringstream oss2;
    oss2 << cloud_header.stamp.nsec;
    string file_name("/home/qinguoyu/radar_slam/" + oss1.str() + "_" + oss2.str() + ".pcd");
    cout << file_name << endl;
    pcl::io::savePCDFileBinary(file_name, *target_pcl);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "radar_odometry");

  RadarOdometry ro;

  ros::spin();
}