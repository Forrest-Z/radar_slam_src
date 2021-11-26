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
  ros::Publisher arbe_after_mappling_pub;

  tf::TransformBroadcaster odom_broadcaster;

  PointCloudA::Ptr arbe_origin_pcl;
  PointCloudA::Ptr arbe_feature_pcl;
  PointCloudI::Ptr arbe_feature_xyzi_pcl;
  PointCloudI::Ptr arbe_feature_xyzi_us_pcl;
  PointCloudI::Ptr arbe_feature_mapped_pcl;
  PointCloudA::Ptr arbe_feature_calc_pcl;
  PointCloudA::Ptr arbe_after_mapping_pcl;

  mutex mBuf;

  Eigen::Quaterniond q_base_map;
  Eigen::Vector3d t_base_map;

  Eigen::Quaterniond q_odo_base;
  Eigen::Vector3d t_odo_base;

  Eigen::Quaterniond q_odo_map;
  Eigen::Vector3d t_odo_map;

  Eigen::Quaterniond q_odo_map_calc;
  Eigen::Vector3d t_odo_map_calc;

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
  vector<PointCloudI::Ptr> key_fram_pcl;

  PointCloudI::Ptr key_pose_surround_t;
  PointCloudI::Ptr key_pose_surround_us_t;
  PointCloudI::Ptr key_pose_surround_exist_t;

  deque<PointCloudI::Ptr> key_frame_surround_exist_list;
  vector<int> key_frame_surround_index_list;

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_key_frame;
  std::vector<int> point_search_ind;
  std::vector<float> point_search_dist;


  PointCloudI::Ptr sub_map_pcl;
  PointCloudI::Ptr sub_map_us_pcl;

  pcl::UniformSampling<PointT> key_pose_us;
  pcl::UniformSampling<PointT> sub_map_us;

  pcl::IterativeClosestPoint<PointT, PointT> icp;

public:
  RadarMapping() :nh("~")
  {
    arbe_feature_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_static_topic, 1,
      &RadarMapping::arbe_feature_cb, this);
    arbe_origin_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_origin_topic, 1,
      &RadarMapping::arbe_origin_cb, this);
    arbe_radar_odometry_sub = nh.subscribe<nav_msgs::Odometry>(arbe_radar_odometry_topic, 1,
      &RadarMapping::arbe_radar_odometry_cb, this);

    arbe_mapping_odometry_pub = nh.advertise<nav_msgs::Odometry>(arbe_mapping_odometry_topic, 100);
    arbe_mapping_path_pub = nh.advertise<nav_msgs::Path>(arbe_mapping_path_topic, 100);
    arbe_mapping_pub = nh.advertise<sensor_msgs::PointCloud2 >(arbe_mapping_topic, 10);
    arbe_after_mappling_pub = nh.advertise<sensor_msgs::PointCloud2 >(arbe_after_mappling_topic, 10);

    q_base_map = Eigen::Quaterniond(1, 0, 0, 0);
    t_base_map = Eigen::Vector3d(0, 0, 0);

    q_odo_base = Eigen::Quaterniond(1, 0, 0, 0);
    t_odo_base = Eigen::Vector3d(0, 0, 0);

    q_odo_map = Eigen::Quaterniond(1, 0, 0, 0);
    t_odo_map = Eigen::Vector3d(0, 0, 0);

    arbe_origin_time = 0;
    arbe_feature_time = 0;
    arbe_radar_odometry_time = 0;

    arbe_feature_new = false;
    arbe_origin_new = false;
    arbe_radar_odometry_new = false;

    key_frame_us.setRadiusSearch(0.2);
    key_pose_us.setRadiusSearch(1.0);
    sub_map_us.setRadiusSearch(0.2);

    curr_pose.x = pre_pose.x = 0;
    curr_pose.y = pre_pose.y = 0;
    curr_pose.z = pre_pose.z = 0;
    curr_pose.intensity = pre_pose.intensity = 0;



    icp.setMaxCorrespondenceDistance(3);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setMaximumIterations(MAX_ICP_NUMBER);

    allocate_memory();

  }

  void allocate_memory()
  {
    arbe_origin_pcl.reset(new PointCloudA());
    arbe_feature_pcl.reset(new PointCloudA());
    arbe_feature_xyzi_pcl.reset(new PointCloudI());
    arbe_feature_xyzi_us_pcl.reset(new PointCloudI());
    arbe_feature_mapped_pcl.reset(new PointCloudI());
    arbe_after_mapping_pcl.reset(new PointCloudA());

    key_pose_t.reset(new PointCloudI());
    key_pose_qt.reset(new PointCloudQT());

    key_pose_surround_t.reset(new PointCloudI());
    key_pose_surround_us_t.reset(new PointCloudI());
    key_pose_surround_exist_t.reset(new PointCloudI());

    sub_map_pcl.reset(new PointCloudI());
    sub_map_us_pcl.reset(new PointCloudI());

    kdtree_key_frame.reset(new pcl::KdTreeFLANN<PointI>());
  }

  void arbe_origin_cb(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros) {
    mBuf.lock();
    arbe_origin_time = arbe_origin_ros->header.stamp.toSec();
    arbe_origin_pcl->clear();
    pcl::fromROSMsg(*arbe_origin_ros, *arbe_origin_pcl);
    arbe_origin_new = true;
    mBuf.unlock();
  }

  void arbe_feature_cb(const sensor_msgs::PointCloud2ConstPtr& arbe_feature_ros) {
    mBuf.lock();
    arbe_feature_time = arbe_feature_ros->header.stamp.toSec();
    arbe_feature_pcl->clear();
    pcl::fromROSMsg(*arbe_feature_ros, *arbe_feature_pcl);
    arbe_feature_new = true;
    mBuf.unlock();
  }

  void arbe_radar_odometry_cb(const nav_msgs::Odometry::ConstPtr& arbe_radar_odometry_ros) {
    mBuf.lock();
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

    Eigen::Quaterniond q_odo_map_tmp = q_base_map * q_odo_base;
    Eigen::Vector3d t_odo_map_tmp = q_base_map * t_odo_base + t_base_map;

    nav_msgs::Odometry arbe_mapping_odometry_ros;
    arbe_mapping_odometry_ros.header = arbe_radar_odometry_ros->header;
    arbe_mapping_odometry_ros.child_frame_id == "mapping";
    arbe_mapping_odometry_ros.pose.pose.orientation.x = q_odo_map_tmp.x();
    arbe_mapping_odometry_ros.pose.pose.orientation.y = q_odo_map_tmp.y();
    arbe_mapping_odometry_ros.pose.pose.orientation.z = q_odo_map_tmp.z();
    arbe_mapping_odometry_ros.pose.pose.orientation.w = q_odo_map_tmp.w();
    arbe_mapping_odometry_ros.pose.pose.position.x = t_odo_map_tmp.x();
    arbe_mapping_odometry_ros.pose.pose.position.y = t_odo_map_tmp.y();
    arbe_mapping_odometry_ros.pose.pose.position.z = t_odo_map_tmp.z();
    arbe_mapping_odometry_pub.publish(arbe_mapping_odometry_ros);
    mBuf.unlock();

  }

  void run()
  {
    if (arbe_feature_new && arbe_feature_new && std::abs(arbe_feature_time - arbe_radar_odometry_time) < 0.005)
    {
      save_data();

      translate_base_to_map();

      extract_surround_key_frame();

      pre_process_feature();

      scan_to_map();



      //     scan2MapOptimization();

      //     saveKeyFramesAndFactor();

      //     correctPoses();

      //     publishTF();

      //     publishKeyPosesAndFrames();

      //     clearCloud();
      //   }

      // }


    };

    void save_data()
    {
      q_odo_map_calc = q_odo_map;
      t_odo_map_calc = t_odo_map;
      *arbe_feature_calc_pcl = *arbe_feature_pcl;
      arbe_feature_new = false; arbe_feature_new = false;
    }

    void scan_to_map()
    {
      icp.setInputSource(arbe_feature_xyzi_us_pcl);
      icp.setInputTarget(sub_map_us_pcl);

      icp.align(*arbe_feature_mapped_pcl);

      // ROS_INFO("calculate_icp : MAX_ICP_NUMBER %d", MAX_ICP_NUMBER);
      ROS_INFO("has converged : %d score: %f", icp.hasConverged(), icp.getFitnessScore());

      std_msgs::Float64 icp_score_msg;
      icp_score_msg.data = (float)icp.getFitnessScore();
      icp_score_pub.publish(icp_score_msg);

      qt_odo_last = icp.getFinalTransformation();
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

    void pre_process_feature()
    {
      pcl::copyPointCloud(*arbe_feature_calc_pcl, *arbe_feature_xyzi_pcl);

      Eigen::Isometry3d  odometry_rt(q_odo_map_calc);
      odometry_rt.pretranslate(t_odo_map_calc);
      pcl::transformPointCloud(*arbe_feature_xyzi_pcl, *arbe_feature_xyzi_pcl, odometry_rt.matrix());

      for (int i = 0; i < arbe_feature_xyzi_pcl->size();i++)
      {
        arbe_feature_xyzi_pcl->points[i].z = 0;
      }
      key_frame_us.setInputCloud(arbe_feature_xyzi_pcl);
      key_frame_us.filter(*arbe_feature_xyzi_us_pcl);
      ROS_INFO("copy_pointcloud : get %d feature point, us get %d us point",
        (int)arbe_feature_pcl->size(), (int)arbe_feature_xyzi_us_pcl->size());
    }

    void translate_base_to_map()
    {
      q_odo_map = q_base_map * q_odo_base;
      t_odo_map = q_base_map * t_odo_base + t_base_map;
    }

    void extract_surround_key_frame()
    {
      if (key_frame_pose_t->size() > 0)
      {
        key_pose_surround_t->clear();
        key_pose_surround_us_t->clear();

        kdtree_key_frame->setInputCloud(key_pose_t);
        kdtreeSurroundingKeyPoses->radiusSearch(curr_pose, KEY_POSE_SEARCH_RADIUS, point_search_ind, point_search_dist);

        for (int index : point_search_ind.)
          key_pose_surround_t->points.push_back(key_pose_t->points[index]);

        key_pose_us.setInputCloud(key_pose_surround_t);
        key_pose_us.filter(*key_pose_surround_us_t);

        for (int i = 0;i < key_frame_surround_exist.size();i++)
        {
          bool exist = false;
          for (int j = 0;j < key_pose_surround_us_t->size();j++)
          {
            if (key_frame_surround_index[i] == (int)key_pose_surround_us_t->points[j].intensity)
            {
              exist = true;
              break;
            }
            if (!exist)
            {
              key_frame_surround_index.erase(key_frame_surround_index.begin() + i);
              key_frame_surround_exist.erase(key_frame_surround_exist.begin() + i);
              --i;
            }
          }
        }

        for (int j = 0;j < key_pose_surround_us_t->size();j++)
        {
          bool exist = false;
          for (int i = 0;i < key_frame_surround_exist.size();i++)
          {
            if (key_frame_surround_index[j] == (int)key_pose_surround_us_t->points[i].intensity)
            {
              exist = true;
              break;
            }
            if (!exist)
            {
              int this_index = (int)key_pose_surround_us_t->points[i].intensity;
              PointQT this_qt = key_pose_surround_t->points[this_index];
              key_frame_surround_index.push_back(this_index);
              key_frame_surround_exist.push_back(key_fram_pcl[this_index];)

            }
          }
        }

        for (int i = 0;i < key_frame_surround_index.size();i++)
        {
          *sub_map += key_frame_surround_exist[i];
        }

        sub_map_us.setInputCloud(sub_map_pcl);
        sub_map_us.filter(*sub_map_us_pcl);
      }
    }


    int main(int argc, char** argv)
    {
      ros::init(argc, argv, "radar_mapping");

      RadarMapping rm;

      ros::Rate rate(200);
      while (ros::ok())
      {
        ros::spinOnce();

        rm.run();

        rate.sleep();
      }
    }