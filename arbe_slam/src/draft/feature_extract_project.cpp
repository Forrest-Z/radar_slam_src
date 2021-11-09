#include "utility.h"

class FeatureExtrace {
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_origin_sub;
  ros::Publisher arbe_project_pub;
  ros::Publisher arbe_outlier_pub;
  ros::Publisher arbe_static_pub;
  ros::Publisher arbe_moving_pub;
  ros::Publisher arbe_object_pub;
  ros::Publisher arbe_object_marker_pub;

  ros::Publisher arbe_ego_doppler_pub;

  PointCloudA::Ptr arbe_origin_pcl;
  PointCloudA::Ptr arbe_project_pcl;
  PointCloudA::Ptr arbe_outlier_pcl;
  PointCloudA::Ptr arbe_static_pcl;
  PointCloudA::Ptr arbe_moving_pcl;
  PointCloudAO::Ptr arbe_object_pcl;

  visualization_msgs::MarkerArray arbe_object_marker_msg;

  std_msgs::Header cloud_header;
  pcl::KdTreeFLANN<PointA>::Ptr kdtree_origin;
  pcl::KdTreeFLANN<PointA>::Ptr kdtree_object;

  vector<int> point_label;
  vector<vector<int>> label_color;

  int label_count;

  vector<float> relate_doppler_list;
  vector<int> static_index_list;
  vector<int> moving_index_list;
  float ego_doppler;

  //ego doppler KF
  float Q;

  float ego_doppler_old;
  float ego_doppler_new;
  float ego_doppler_pre;
  float ego_doppler_est;

  float P_old;
  float P_pre;
  float P_new;

  float K;
  float R;

  float A;
  float H;

  bool ego_doppler_init_flag;

  visualization_msgs::Marker arbe_object_marker_clear;


public:
  FeatureExtrace() :nh("~") {
    arbe_origin_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_origin_topic, 1,
      &FeatureExtrace::pointcloud_callback, this);
    arbe_project_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_project_topic, 1);
    arbe_outlier_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_outlier_topic, 1);
    arbe_static_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_static_topic, 1);
    arbe_moving_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_moving_topic, 1);
    arbe_object_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_object_topic, 1);
    arbe_object_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(arbe_object_marker_topic, 1);

    arbe_ego_doppler_pub = nh.advertise<std_msgs::Float64>(arbe_ego_doppler_topic, 1);

    arbe_object_marker_clear.header = cloud_header;
    arbe_object_marker_clear.ns = "arbe_object";
    // arbe_object_marker.id = label;
    arbe_object_marker_clear.type = visualization_msgs::Marker::CUBE;
    arbe_object_marker_clear.action = visualization_msgs::Marker::DELETEALL;

    nh.getParam("max_segment_distance", MAX_SEGMENT_DISTANCE);
    nh.getParam("min_segment_distance", MIN_SEGMENT_DISTANCE);
    nh.getParam("min_segment_number", MIN_SEGMENT_NUMBER);
    nh.getParam("max_segment_doppler_diff", MAX_SEGMENT_DOPPLER_THRE);
    nh.getParam("search_radius_rate", SEARCH_RAIUS_RATE);
    nh.getParam("static_moving_thre", STATIC_MOVING_THRE);

    allocate_memory();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))//Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
    cout << "init!" << endl;
  }

  void allocate_memory() {
    arbe_origin_pcl.reset(new PointCloudA());
    arbe_project_pcl.reset(new PointCloudA());
    arbe_outlier_pcl.reset(new PointCloudA());
    arbe_static_pcl.reset(new PointCloudA());
    arbe_moving_pcl.reset(new PointCloudA());
    arbe_object_pcl.reset(new PointCloudAO());

    kdtree_origin.reset(new pcl::KdTreeFLANN<PointA>());
    inti_ego_doppler_kf();
  }

  void inti_ego_doppler_kf()
  {
    Q = 1e-6;

    ego_doppler_old = 0;
    ego_doppler_new = 0;
    ego_doppler_pre = 0;
    ego_doppler_est = 0;

    P_old = 1;
    P_pre = 0;
    P_new = 0;

    K = 0;
    R = 0.1 * 0.1;

    A = 0;
    H = 0;

    ego_doppler_init_flag = false;
  }

  void ego_doppler_kf()
  {
    ego_doppler_pre = A * ego_doppler_old;
    P_pre = A * P_old + Q;

    K = P_pre / (P_pre + R);
    ego_doppler_est = ego_doppler_pre + K * (ego_doppler_new - H * ego_doppler_pre);
    P_new = (1 - K * H) * P_pre;

    P_old = P_new;
    ego_doppler_old = ego_doppler_est;
    ego_doppler = ego_doppler_est;

    std_msgs::Float64 ego_doppler_msg;
    ego_doppler_msg.data = (double)ego_doppler;
    arbe_ego_doppler_pub.publish(ego_doppler_msg);
  }

  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {
    clock_t start = clock();

    copy_pointcloud(arbe_origin_ros);
    clock_t end_copy = clock();

    project_pointcloud();
    clock_t end_project = clock();

    calculate_ego_doppler();
    clock_t end_doppler = clock();

    segment_pointcloud();
    clock_t end_segment = clock();

    publish_pointcloud();
    clock_t end_publish = clock();

    double duration_copy = (double)(end_copy - start) / CLOCKS_PER_SEC;
    double duration_project = (double)(end_project - end_copy) / CLOCKS_PER_SEC;
    double duration_doppler = (double)(end_doppler - end_copy) / CLOCKS_PER_SEC;
    double duration_segment = (double)(end_segment - end_doppler) / CLOCKS_PER_SEC;
    double duration_publish = (double)(end_publish - end_segment) / CLOCKS_PER_SEC;

    double duration_total = duration_copy + duration_project + duration_doppler + duration_segment + duration_publish;

    ROS_INFO("duration: %f + %f + %f + %f+ %f = %f.",
      duration_copy, duration_project, duration_doppler, duration_segment, duration_publish, duration_total);
  }

  void copy_pointcloud(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {
    ROS_INFO("copy_pointcloud : get %d origin ROS point!", arbe_origin_ros->width);
    cloud_header = arbe_origin_ros->header;
    cloud_header.stamp = ros::Time::now();
    pcl::fromROSMsg(*arbe_origin_ros, *arbe_origin_pcl);
  }

  void project_pointcloud()
  {
    unordered_map<pair<int, int>, pair<int, float>, PairHash>  origin_point_map;
    for (int i = 0;i < arbe_origin_pcl->size();i++)
    {
      PointA point = arbe_origin_pcl->points[i];
      pair<int, int> row_col((int)point.azimuth_bin, (int)point.elevation_bin);

      if (origin_point_map.find(row_col) == origin_point_map.end())
      {
        // pair<int, float> point_info(i, point.range);
        pair<int, float> point_info(i, point.power);
        origin_point_map.emplace(row_col, point_info);
      }
      else {
        // if (point.range < origin_point_map[row_col].second)
        if (point.power < origin_point_map[row_col].second)
        {
          // pair<int, float> point_info(i, point.range);
          pair<int, float> point_info(i, point.power);
          origin_point_map.emplace(row_col, point_info);
        }
      }
    }

    for (auto iter : origin_point_map)
    {
      PointA point = arbe_origin_pcl->points[iter.second.first];
      arbe_project_pcl->push_back(point);
    }

    kdtree_origin->setInputCloud(arbe_project_pcl);

    ROS_INFO("project_pointcloud : origin pointcloud number %d, project pointcloud number %d.",
      arbe_origin_pcl->size(), arbe_project_pcl->size());
  }

  float ceres_solver_ego_doppler()
  {
    clock_t start, end;
    start = clock();
    //ceres::LossFunction *loss_function = NULL;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(motion_estimate, 2);

    for (int i = 0; i < int(arbe_project_pcl->size() / 5);i++)
    {
      PointA point = arbe_project_pcl->points[i * 5];
      ceres::CostFunction* cost_function = RadarDopplerFactor::Create
      ((double)point.doppler, (double)point.azimuth, (double)(point.elevation + 0.1));
      problem.AddResidualBlock(cost_function, loss_function, motion_estimate);

    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 4;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    end = clock();
    float duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("ceres_solver_ego_doppler : velocity_y %f, yaw %f, duration %f s.",
      motion_estimate[0], motion_estimate[1], duration);
  }

  float calculate_ego_doppler()
  {
    unordered_map<int, float, IntHash> doppler_counter;

    relate_doppler_list.clear();

    for (size_t i = 0; i < arbe_project_pcl->size();i++)
    {
      PointA point = arbe_project_pcl->points[i];
      float relate_doppler = point.doppler / (cos(point.azimuth) * cos(point.elevation));
      int relate_doppler_ten = (int)(10 * relate_doppler);
      relate_doppler_list.push_back(relate_doppler);
      if (point.range > MIN_CALCULATE_EGO_DOPPLER_RADIUS)
      {
        if (doppler_counter.find(relate_doppler_ten) == doppler_counter.end())
        {
          doppler_counter.emplace(relate_doppler_ten, abs(point.azimuth));
        }
        else {
          doppler_counter[relate_doppler_ten] += abs(point.azimuth);
        }
      }
    }
    PointA point_0 = arbe_project_pcl->points[0];
    int ego_doppler_ten = (int)(10 * point_0.doppler / (cos(point_0.azimuth) * cos(point_0.elevation)));
    int max_counter = 0;

    for (auto iter = doppler_counter.begin(); iter != doppler_counter.end(); ++iter)
    {
      if (iter->second > max_counter)
      {
        ego_doppler_ten = iter->first;
        max_counter = iter->second;
      }
    }

    float ego_doppler_sta = (float)ego_doppler_ten / 10.;

    if (ego_doppler_init_flag)
    {
      ego_doppler_new = ego_doppler_sta;
      ego_doppler_kf();

    }
    else {
      ego_doppler = ego_doppler_old = ego_doppler_sta;
    }

    ROS_INFO("calculate_ego_doppler : ego doppler new: %f,ego doppler estimate: %f, max counter : %d, total counter : %d.",
      ego_doppler_sta, ego_doppler, max_counter, (int)arbe_project_pcl->size());
    doppler_counter.clear();
  }

  void segment_pointcloud()
  {
    moving_index_list.clear();
    for (size_t i = 0; i < arbe_project_pcl->size();i++)
    {
      if (abs(relate_doppler_list[i] - ego_doppler) > STATIC_MOVING_THRE)
      {
        moving_index_list.push_back(i);
        // ROS_INFO("point %d, doppler diff : %f", (int)i, abs(relate_doppler_list[i] - ego_doppler));
      }
    }

    ROS_INFO("segment_pointcloud : origin moving points : %d!", (int)moving_index_list.size());

    label_count = 1;
    point_label.clear();
    point_label.resize(arbe_project_pcl->size(), 0);
    for (int index : moving_index_list)
    {
      if (point_label[index] == 0)
      {
        label_pointcloud(index);
        point_label[index] = 1;
        ROS_DEBUG("segment_pointcloud : label point %d.", index);
      }
    }
  }

  void label_pointcloud(int index) {

    vector<int> point_label_tmp;
    point_label_tmp.resize(arbe_project_pcl->size(), 0);

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
      PointA point_center = arbe_project_pcl->points[centrer_index];
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

            PointA point_search = arbe_project_pcl->points[search_index];
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
      ;
      if (!get_object_point(label, segmentation_index))
      {
        label_count += 1;
      }
    }

    for (int i : segmentation_index)
    {
      point_label[i] = label;
    }
    ROS_DEBUG("abel_pointcloud : label object %d with %d points.", label, (int)segmentation_index.size());
  }

  bool get_object_point(int label, vector<int>& segmentation_index)
  {
    vector<float> x_list;
    vector<float> y_list;
    vector<float> z_list;
    vector<float> doppler_list;
    vector<float> power_list;

    int final_label = label;

    int arbe_moving_pcl_start = arbe_moving_pcl->size();

    for (int i : segmentation_index)
    {
      PointA point = arbe_project_pcl->points[i];
      x_list.push_back(point.x);
      y_list.push_back(point.y);
      z_list.push_back(point.z);
      doppler_list.push_back(point.doppler);
      power_list.push_back(point.power);
      arbe_moving_pcl->push_back(point);
    }
    int arbe_moving_pcl_end = arbe_moving_pcl->size();

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
    float width = x_max > x_min ? (x_max - x_min) : 0.5;
    float y_mean = (y_min + y_max) / 2;
    float length = y_max > y_min ? (y_max - y_min) : 0.5;
    float z_mean = (z_min + z_max) / 2;
    float high = z_max > z_min ? (z_max - z_min) : 0.5;

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

    arbe_object.quantity = x_list.size();
    arbe_object.label = label;

    bool merge_flag = false;
    for (size_t i = 0;i < arbe_object_pcl->size(); i++)
    {
      PointAO arbe_object_search = arbe_object_pcl->points[i];

      if ((abs(arbe_object_search.x - arbe_object.x) - abs(arbe_object_search.width + arbe_object.width) / 2 < MAX_OBJECT_DISTANCE
        && abs(arbe_object_search.y - arbe_object.y) - abs(arbe_object_search.length + arbe_object.length) / 2 < MAX_OBJECT_DISTANCE
        && abs(arbe_object_search.z - arbe_object.z) - abs(arbe_object_search.high + arbe_object.high) / 2 < MAX_OBJECT_DISTANCE
        && abs(arbe_object_search.doppler - arbe_object.doppler) < Min_OBJECT_DOPPLER)
        | ((abs(arbe_object_search.x - arbe_object.x) - abs(arbe_object_search.width + arbe_object.width) / 2 < 0
          && abs(arbe_object_search.y - arbe_object.y) - abs(arbe_object_search.length + arbe_object.length) / 2 < 0
          && abs(arbe_object_search.z - arbe_object.z) - abs(arbe_object_search.high + arbe_object.high) / 2 < 0)
          && abs(arbe_object_search.doppler - arbe_object.doppler) < MAX_OBJECT_DOPPLER))
      {
        PointAO arbe_object_merge;
        arbe_object_merge.x = (arbe_object.x * arbe_object.quantity
          + arbe_object_search.x * arbe_object_search.quantity)
          / (arbe_object.quantity + arbe_object_search.quantity);

        arbe_object_merge.y = (arbe_object.y * arbe_object.quantity
          + arbe_object_search.y * arbe_object_search.quantity)
          / (arbe_object.quantity + arbe_object_search.quantity);

        arbe_object_merge.z = (arbe_object.z * arbe_object.quantity
          + arbe_object_search.z * arbe_object_search.quantity)
          / (arbe_object.quantity + arbe_object_search.quantity);

        arbe_object_merge.doppler = (arbe_object.doppler * arbe_object.quantity
          + arbe_object_search.doppler * arbe_object_search.quantity)
          / (arbe_object.quantity + arbe_object_search.quantity);

        arbe_object_merge.power = max(arbe_object.power, arbe_object_search.power);

        arbe_object_merge.width = max(arbe_object.x + arbe_object.width / 2,
          arbe_object_search.x + arbe_object_search.width / 2)
          - min(arbe_object.x - arbe_object.width / 2,
            arbe_object_search.x - arbe_object_search.width / 2);

        arbe_object_merge.length = max(arbe_object.y + arbe_object.length / 2,
          arbe_object_search.y + arbe_object_search.length / 2)
          - min(arbe_object.y - arbe_object.length / 2,
            arbe_object_search.y - arbe_object_search.length / 2);

        arbe_object_merge.high = max(arbe_object.z + arbe_object.high / 2,
          arbe_object_search.z + arbe_object_search.high / 2)
          - min(arbe_object.z - arbe_object.high / 2,
            arbe_object_search.z - arbe_object_search.high / 2);

        arbe_object_merge.quantity = arbe_object.quantity + arbe_object_search.quantity;
        arbe_object_merge.label = arbe_object_search.label;

        arbe_object_pcl->points[i] = arbe_object_merge;
        merge_flag = true;
        final_label = int(i);
        cout << "input label:" << label << ", final label : " << final_label << endl;
        break;
      }
    }


    if (!merge_flag)
    {
      arbe_object_pcl->push_back(arbe_object);
    }

    for (int i = arbe_moving_pcl_start;i < arbe_moving_pcl_end;i++)
    {
      arbe_moving_pcl->points[i].power_value = final_label;
    }

    return merge_flag;
  }

  void publish_pointcloud()
  {
    for (int i = 0;i < arbe_project_pcl->size();i++)
    {
      int label = point_label[i];
      if (label == OUTLIER_LABEL)
      {
        arbe_outlier_pcl->push_back(arbe_project_pcl->points[i]);
      }
      else if (label != 0)
      {
        arbe_moving_pcl->push_back(arbe_project_pcl->points[i]);
        arbe_moving_pcl->back().power_value = label;
        arbe_moving_pcl->back().power = relate_doppler_list[i];
        // ROS_DEBUG("publish_pointcloud : point %d, doppler diff : %f", (int)i, abs(relate_doppler_list[i] - ego_doppler));
      }
      else {
        PointA point = arbe_project_pcl->points[i];
        arbe_static_pcl->push_back(point);
        arbe_static_pcl->back().power = relate_doppler_list[i];
      }
    }

    for (size_t i = 0;i < arbe_object_pcl->size(); i++)
    {

      PointAO arbe_object = arbe_object_pcl->points[i];
      visualization_msgs::Marker arbe_object_marker;
      arbe_object_marker.header = cloud_header;
      arbe_object_marker.ns = "arbe_object";
      arbe_object_marker.id = int(i);
      arbe_object_marker.type = visualization_msgs::Marker::CUBE;
      arbe_object_marker.action = visualization_msgs::Marker::ADD;

      arbe_object_marker.pose.position.x = arbe_object.x;
      arbe_object_marker.pose.position.y = arbe_object.y;
      arbe_object_marker.pose.position.z = arbe_object.z;
      arbe_object_marker.pose.orientation.w = 1;

      arbe_object_marker.scale.x = arbe_object.width;
      arbe_object_marker.scale.y = arbe_object.length;
      arbe_object_marker.scale.z = arbe_object.high;

      arbe_object_marker.color.r = 1.0;
      arbe_object_marker.color.g = 1.0;
      arbe_object_marker.color.b = 1.0;
      arbe_object_marker.color.a = 0.6;

      arbe_object_marker.lifetime = ros::Duration();

      arbe_object_marker_msg.markers.push_back(arbe_object_marker);
    }

    ROS_INFO("publish_pointcloud : segment %d segment point into %d objects, get %d outlier!",
      (int)arbe_moving_pcl->size(), (int)label_count, (int)arbe_outlier_pcl->size());

    sensor_msgs::PointCloud2 arbe_moving_ros;
    pcl::toROSMsg(*arbe_moving_pcl, arbe_moving_ros);
    arbe_moving_ros.header = cloud_header;
    arbe_moving_pub.publish(arbe_moving_ros);

    sensor_msgs::PointCloud2 arbe_outlier_ros;
    pcl::toROSMsg(*arbe_outlier_pcl, arbe_outlier_ros);
    arbe_outlier_ros.header = cloud_header;
    arbe_outlier_pub.publish(arbe_outlier_ros);


    sensor_msgs::PointCloud2 arbe_static_ros;
    pcl::toROSMsg(*arbe_static_pcl, arbe_static_ros);
    arbe_static_ros.header = cloud_header;
    arbe_static_pub.publish(arbe_static_ros);

    sensor_msgs::PointCloud2 arbe_object_ros;
    pcl::toROSMsg(*arbe_object_pcl, arbe_object_ros);
    arbe_object_ros.header = cloud_header;
    arbe_object_pub.publish(arbe_object_ros);

    arbe_object_marker_pub.publish(arbe_object_marker_msg);
    arbe_object_marker_msg.markers.clear();
    arbe_object_marker_msg.markers.push_back(arbe_object_marker_clear);

    arbe_object_pcl->clear();
    arbe_project_pcl->clear();
    arbe_outlier_pcl->clear();
    arbe_moving_pcl->clear();
    arbe_static_pcl->clear();
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_extrace");

  FeatureExtrace fe;

  ros::spin();
}


