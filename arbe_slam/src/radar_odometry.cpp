#include "utility.h"

class RadarOdometry
{
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_feature_sub;
  ros::Publisher arbe_radar_odometry_pub;
  ros::Publisher arbe_radar_path_pub;
  ros::Publisher arbe_feature_xyz_us_old_icp_pub;
  ros::Publisher icp_score_pub;
  tf::TransformBroadcaster odom_broadcaster;

  PointCloudA::Ptr arbe_feature_pcl;
  PointCloudA::Ptr arbe_feature_pcl_old;

  PointCloudT::Ptr arbe_feature_xyz_pcl;
  PointCloudT::Ptr arbe_feature_xyz_old_pcl;
  PointCloudT::Ptr arbe_feature_xyz_old_icp_pcl;

  PointCloudT::Ptr arbe_feature_xyz_us_pcl;
  PointCloudT::Ptr arbe_feature_xyz_us_old_pcl;
  PointCloudT::Ptr arbe_feature_xyz_us_old_icp_pcl;


  std_msgs::Header cloud_header;

  bool init;

  string frame_if;
  string child_frame_id;

  pcl::IterativeClosestPoint<PointT, PointT> icp;

  Eigen::Quaterniond icp_q;
  Eigen::Vector3d icp_a;
  Eigen::Vector3d icp_t;

  Eigen::Quaterniond curr_q;
  Eigen::Vector3d curr_t;

  nav_msgs::Path radar_path;

  pcl::UniformSampling<PointT> US;

  Eigen::Matrix4f estimate_tans;


public:
  RadarOdometry() :nh("~") {
    arbe_feature_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_static_front_topic, 1,
      &RadarOdometry::arbe_feature_cb, this);

    arbe_radar_odometry_pub = nh.advertise<nav_msgs::Odometry>(arbe_radar_odometry_topic, 100);
    arbe_radar_path_pub = nh.advertise<nav_msgs::Path>(arbe_radar_path_topic, 100);
    arbe_feature_xyz_us_old_icp_pub = nh.advertise<sensor_msgs::PointCloud2 >(arbe_feature_xyz_us_old_icp_topic, 10);
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

    US.setRadiusSearch(0.01f);//设置滤波时创建球体的半径


    icp_q = Eigen::Quaterniond(0, 0, 0, 1);
    icp_a = Eigen::Vector3d(0, 0, 0);
    icp_t = Eigen::Vector3d(0, 0, 0);

    curr_q = Eigen::Quaterniond(0, 0, 0, 1);
    curr_t = Eigen::Vector3d(0, 0, 0);

    radar_path.header.stamp = cloud_header.stamp;
    radar_path.header.frame_id = "base_link";

    estimate_tans = Eigen::Matrix4f::Identity();

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

    arbe_feature_xyz_pcl.reset(new PointCloudT());
    arbe_feature_xyz_old_pcl.reset(new PointCloudT());
    arbe_feature_xyz_old_icp_pcl.reset(new PointCloudT());

    arbe_feature_xyz_us_pcl.reset(new PointCloudT());
    arbe_feature_xyz_us_old_pcl.reset(new PointCloudT());
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
      // PointCloudT::Ptr arbe_feature_xyz_us_old_est_pcl(new PointCloudT);
      // pcl::transformPointCloud(*arbe_feature_xyz_us_old_pcl, *arbe_feature_xyz_us_old_est_pcl, estimate_tans);
      // icp.setInputSource(arbe_feature_xyz_us_old_est_pcl);

      icp.setInputSource(arbe_feature_xyz_us_old_pcl);
      icp.setInputTarget(arbe_feature_xyz_us_pcl);
      icp.align(*arbe_feature_xyz_us_old_icp_pcl);

      ROS_INFO("calculate_icp : MAX_ICP_NUMBER %d", MAX_ICP_NUMBER);

      ROS_INFO("has converged : %d score: %f", icp.hasConverged(), icp.getFitnessScore());
      // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
      //   icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;

      std_msgs::Float64 icp_score_msg;
      icp_score_msg.data = (float)icp.getFitnessScore();
      icp_score_pub.publish(icp_score_msg);

      float x, y, z, roll, pitch, yaw;
      Eigen::Affine3f icp_trans;
      Eigen::Matrix4f final_tans;

      // final_tans = icp.getFinalTransformation() * estimate_tans;
      // estimate_tans = final_tans;
      // icp_trans.matrix() = final_tans;

      icp_trans = icp.getFinalTransformation();

      pcl::getTranslationAndEulerAngles(icp_trans, x, y, z, roll, pitch, yaw);

      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
      icp_q = yawAngle * pitchAngle * rollAngle;

      icp_a = Eigen::Vector3d(roll, pitch, yaw);
      icp_t = Eigen::Vector3d(x, y, z);

      curr_t = curr_t + curr_q * icp_t;
      curr_q = curr_q * icp_q;

      // cout << "output : "
      //   << icp_q.x() << "\t" << icp_q.y() << "\t" << icp_q.z() << "\t" << icp_q.w() << "\n"
      //   << icp_a.x() << "\t" << icp_a.y() << "\t" << icp_a.z() << "\n"
      //   << icp_t.x() << "\t" << icp_t.y() << "\t" << icp_t.z() << "\n"
      //   << curr_q.x() << "\t" << curr_q.y() << "\t" << curr_q.z() << "\t" << curr_q.w() << "\n"
      //   << curr_t.x() << "\t" << curr_t.y() << "\t" << curr_t.z() << endl;

      // Eigen::Matrix4f T2 = icp.getFinalTransformation();
      // Eigen::Matrix3f rotation_matrix1 = T2.block<3, 3>(0, 0);
      // Eigen::Vector3f t1 = T2.topRightCorner(3, 1);

      // Eigen::Vector3f eulerAngle = rotation_matrix1.eulerAngles(2, 1, 0);

      // // cout << "rotation_matrix1\n" << rotation_matrix1 << endl; 
      // cout << "eulerAngle\n" << eulerAngle.transpose() << endl;
      // cout << "t1\n" << t1 << endl;
    }
    else {
      init = true;
    }

    *arbe_feature_xyz_us_old_pcl = *arbe_feature_xyz_us_pcl;
  }

  void publish_odometry()
  {
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(curr_q.x(), curr_q.y(), curr_q.z(), curr_q.w())).getRPY(roll, pitch, yaw);

    // odometry
    std_msgs::Header header;
    header.stamp = cloud_header.stamp;
    header.frame_id = "base_link";

    nav_msgs::Odometry radar_odometry;
    radar_odometry.header = header;
    radar_odometry.child_frame_id = "odometry";
    radar_odometry.pose.pose.orientation.x = curr_q.x();
    radar_odometry.pose.pose.orientation.y = curr_q.y();
    radar_odometry.pose.pose.orientation.z = curr_q.z();
    radar_odometry.pose.pose.orientation.w = curr_q.w();
    radar_odometry.pose.pose.position.x = curr_t.x();
    radar_odometry.pose.pose.position.y = curr_t.y();
    radar_odometry.pose.pose.position.z = curr_t.z();
    radar_odometry.twist.twist.angular.x = roll;
    radar_odometry.twist.twist.angular.y = pitch;
    radar_odometry.twist.twist.angular.z = yaw;
    radar_odometry.twist.twist.linear.x = curr_t.x();
    radar_odometry.twist.twist.linear.y = curr_t.y();
    radar_odometry.twist.twist.linear.z = curr_t.z();
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
    odom_trans.transform.translation.x = curr_t.x();
    odom_trans.transform.translation.y = curr_t.y();
    odom_trans.transform.translation.z = curr_t.z();
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
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "radar_odometry");

  RadarOdometry ro;

  ros::spin();
}