#include "utility.h"

class RadarOdometry
{
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_static_front_sub;
  ros::Publisher arbe_radar_odometry_pub;
  ros::Publisher arbe_radar_path_pub;
  tf::TransformBroadcaster odom_broadcaster;

  PointCloudA::Ptr arbe_static_front_pcl;
  PointCloudA::Ptr arbe_static_front_pcl_old;

  PointCloudT::Ptr arbe_static_xyz_pcl;
  PointCloudT::Ptr arbe_static_xyz_pcl_old;

  std_msgs::Header cloud_header;

  int init;

  string frame_if;
  string child_frame_id;

  pcl::IterativeClosestPoint<PointT, PointT> icp;

  Eigen::Quaterniond icp_q;
  Eigen::Vector3d icp_a;
  Eigen::Vector3d icp_t;

  Eigen::Quaterniond curr_q;
  Eigen::Vector3d curr_t;


public:
  RadarOdometry() :nh("~") {
    arbe_static_front_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_static_front_topic, 1,
      &RadarOdometry::arbe_static_front_cb, this);

    arbe_radar_odometry_pub = nh.advertise<nav_msgs::Odometry>(arbe_radar_odometry_topic, 100);
    arbe_radar_path_pub = nh.advertise<nav_msgs::Path>(arbe_radar_path_topic, 100);

    nh.getParam("frame_if", frame_if);
    nh.getParam("child_frame_id", child_frame_id);

    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp_q = Eigen::Quaterniond(1, 0, 0, 0);
    icp_a = Eigen::Vector3d(0, 0, 0);
    icp_t = Eigen::Vector3d(0, 0, 0);

    curr_q = Eigen::Quaterniond(1, 0, 0, 0);
    curr_t = Eigen::Vector3d(0, 0, 0);


    init = 0;

    allocate_memory();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))//Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
    cout << "RadarOdometry init!" << endl;

  }

  void allocate_memory() {
    arbe_static_front_pcl.reset(new PointCloudA());
    arbe_static_front_pcl_old.reset(new PointCloudA());

    arbe_static_xyz_pcl.reset(new PointCloudT());
    arbe_static_xyz_pcl_old.reset(new PointCloudT());

  }

  void arbe_static_front_cb(const sensor_msgs::PointCloud2ConstPtr& arbe_static_front_ros)
  {
    copy_pointcloud(arbe_static_front_ros);
    calculate_icp();
    publish_odometry();
  }

  void copy_pointcloud(const sensor_msgs::PointCloud2ConstPtr& arbe_static_front_ros)
  {
    // ROS_INFO("copy_pointcloud : get %d static front ROS point!", ((int)arbe_static_front_ros->width);
    cloud_header = arbe_static_front_ros->header;
    pcl::fromROSMsg(*arbe_static_front_ros, *arbe_static_front_pcl);
    pcl::copyPointCloud(*arbe_static_front_pcl, *arbe_static_xyz_pcl);
  }

  void calculate_icp()
  {
    if (init)
    {
      icp.setInputSource(arbe_static_xyz_pcl_old);
      icp.setInputTarget(arbe_static_xyz_pcl);
      PointCloudT Final;
      icp.align(Final);
      // ROS_INFO("has converged : %d score: %f", icp.hasConverged(), icp.getFitnessScore());
      std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
      // std::cout << icp.getFinalTransformation() << std::endl;


      float x, y, z, roll, pitch, yaw;
      Eigen::Affine3f icp_trans;
      icp_trans = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
      pcl::getTranslationAndEulerAngles(icp_trans, x, y, z, roll, pitch, yaw);

      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
      icp_q = yawAngle * pitchAngle * rollAngle;


      icp_a = Eigen::Vector3d(roll, pitch, yaw);
      icp_t = Eigen::Vector3d(x, y, z);

      curr_t = curr_t + curr_q * icp_t;
      curr_q = curr_q * icp_q;

      // Eigen::Matrix4f T2 = icp.getFinalTransformation();
      // Eigen::Matrix3f rotation_matrix1 = T2.block<3, 3>(0, 0);
      // Eigen::Vector3f t1 = T2.topRightCorner(3, 1);

      // Eigen::Vector3f eulerAngle = rotation_matrix1.eulerAngles(2, 1, 0);

      // // cout << "rotation_matrix1\n" << rotation_matrix1 << endl; 
      // cout << "eulerAngle\n" << eulerAngle.transpose() << endl;
      // cout << "t1\n" << t1 << endl;
    }
    else {
      init = 1;
    }

    *arbe_static_xyz_pcl_old = *arbe_static_xyz_pcl;
  }


  void publish_odometry()
  {
    // publish odometry
    // geometry_msgs::Quaternion odom_quat;
    // odom_quat.x = curr_q.x();
    // odom_quat.y = curr_q.y();
    // odom_quat.z = curr_q.z();
    // odom_quat.w = curr_q.w();


    // geometry_msgs::Quaternion icp_tf_q = tf::createQuaternionMsgFromRollPitchYaw
    // (icp_a.x(), icp_a.y(), icp_a.z());

    // geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
    // (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

    // odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    // odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
    // odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
    // odomAftMapped.pose.pose.orientation.z = geoQuat.x;
    // odomAftMapped.pose.pose.orientation.w = geoQuat.w;
    // odomAftMapped.pose.pose.position.x = transformAftMapped[3];
    // odomAftMapped.pose.pose.position.y = transformAftMapped[4];
    // odomAftMapped.pose.pose.position.z = transformAftMapped[5];
    // odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
    // odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
    // odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
    // odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
    // odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
    // odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
    // pubOdomAftMapped.publish(odomAftMapped);

    // aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
    // aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    // aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
    // tfBroadcaster.sendTransform(aftMappedTrans);

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(curr_q.x(), curr_q.y(), curr_q.z(), curr_q.w())).getRPY(roll, pitch, yaw);

    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "base_link";
    laserOdometry.child_frame_id = "odometry";
    laserOdometry.header.stamp = cloud_header.stamp;
    laserOdometry.pose.pose.orientation.x = curr_q.x();
    laserOdometry.pose.pose.orientation.y = curr_q.y();
    laserOdometry.pose.pose.orientation.z = curr_q.z();
    laserOdometry.pose.pose.orientation.w = curr_q.w();
    laserOdometry.pose.pose.position.x = curr_t.x();
    laserOdometry.pose.pose.position.y = curr_t.y();
    laserOdometry.pose.pose.position.z = curr_t.z();
    laserOdometry.twist.twist.angular.x = roll;
    laserOdometry.twist.twist.angular.y = pitch;
    laserOdometry.twist.twist.angular.z = yaw;
    laserOdometry.twist.twist.linear.x = curr_t.x();
    laserOdometry.twist.twist.linear.y = curr_t.y();
    laserOdometry.twist.twist.linear.z = curr_t.z();
    arbe_radar_odometry_pub.publish(laserOdometry);



    tf::StampedTransform odom_trans;
    odom_trans.stamp_ = cloud_header.stamp;
    odom_trans.frame_id_ = "base_link";
    odom_trans.child_frame_id_ = "odometry";
    odom_trans.setRotation(tf::Quaternion(curr_q.x(), curr_q.y(), curr_q.z(), curr_q.w()));
    odom_trans.setOrigin(tf::Vector3(curr_t.x(), curr_t.y(), curr_t.z()));
    odom_broadcaster.sendTransform(odom_trans);


    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = cloud_header.stamp;
    // odom_trans.header.frame_id = "base_link";
    // odom_trans.child_frame_id = "odometry";

    // odom_trans.transform.translation.x = curr_t.x();
    // odom_trans.transform.translation.y = curr_t.y();
    // odom_trans.transform.translation.z = curr_t.z();
    // odom_trans.transform.rotation.x = curr_q.x();
    // odom_trans.transform.rotation.x = curr_q.y();
    // odom_trans.transform.rotation.x = curr_q.z();
    // odom_trans.transform.rotation.x = curr_q.w();
    // odom_broadcaster.sendTransform(odom_trans);


    geometry_msgs::PoseStamped laserPose;
    laserPose.header = laserOdometry.header;
    laserPose.pose = laserOdometry.pose.pose;
    nav_msgs::Path laserPath;
    laserPath.header.stamp = laserOdometry.header.stamp;
    laserPath.poses.push_back(laserPose);
    laserPath.header.frame_id = "base_link";
    // arbe_radar_path_pub.publish(laserPath);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "radar_odometry");

  RadarOdometry ro;

  ros::spin();
}