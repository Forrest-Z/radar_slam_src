#include "utility.h"

class FeatureExtrace {
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_origin_sub;
  ros::Subscriber dbw_sub;
  ros::Publisher arbe_outlier_pub;
  ros::Publisher arbe_static_pub;
  ros::Publisher arbe_static_front_pub;
  ros::Publisher arbe_moving_pub;
  ros::Publisher arbe_moving_front_pub;
  ros::Publisher arbe_object_pub;
  ros::Publisher arbe_object_marker_pub;

  ros::Publisher arbe_os_cfar_pub;
  ros::Publisher arbe_project_image_pub;

  ros::Publisher arbe_ego_doppler_pub;
  ros::Publisher arbe_ego_doppler_kf_pub;
  ros::Publisher arbe_ego_doppler_x_pub;
  ros::Publisher arbe_ego_doppler_y_pub;

  PointCloudA::Ptr arbe_origin_pcl;
  PointCloudA::Ptr arbe_segment_pcl;
  PointCloudA::Ptr arbe_outlier_pcl;
  PointCloudA::Ptr arbe_static_pcl;
  PointCloudA::Ptr arbe_static_front_pcl;
  PointCloudA::Ptr arbe_moving_pcl;
  PointCloudA::Ptr arbe_moving_front_pcl;
  PointCloudAO::Ptr arbe_object_pcl;

  PointCloudI::Ptr arbe_polar_pcl;
  PointCloudA::Ptr arbe_os_cfar_pcl;
  sensor_msgs::CompressedImage arbe_project_image_msg;

  visualization_msgs::MarkerArray arbe_object_marker_msg;

  std_msgs::Header cloud_header;
  pcl::KdTreeFLANN<PointA>::Ptr kdtree_origin;
  pcl::KdTreeFLANN<PointA>::Ptr kdtree_object;
  pcl::KdTreeFLANN<PointI>::Ptr kdtree_polar;

  vector<int> point_label;
  vector<vector<int>> label_color;

  int label_count;

  vector<float> relate_doppler_list;
  vector<int> static_index_list;
  vector<int> moving_index_list;
  float ego_doppler;

  //ego doppler KF
  double Q;

  double ego_doppler_old;
  double ego_doppler_new;
  double ego_doppler_pre;
  double ego_doppler_est;

  double P_old;
  double P_pre;
  double P_new;

  double K;
  double R;

  double A;
  double H;

  bool ego_doppler_init_flag;

  visualization_msgs::Marker arbe_object_marker_clear;

  cv::Mat arbe_project_image;

  float vehicle_yaw;


public:
  FeatureExtrace() :nh("~") {
    arbe_origin_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_origin_topic, 1,
      &FeatureExtrace::pointcloud_callback, this);
    dbw_sub = nh.subscribe<arbe_slam::CANROS>(dbw_topic, 1,
      &FeatureExtrace::dbw_callback, this);

    arbe_outlier_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_outlier_topic, 1);
    arbe_static_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_static_topic, 1);
    arbe_static_front_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_static_front_topic, 1);
    arbe_moving_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_moving_topic, 1);
    arbe_moving_front_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_moving_front_topic, 1);
    arbe_object_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_object_topic, 1);
    arbe_object_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(arbe_object_marker_topic, 1);

    arbe_os_cfar_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_os_cfar_topic, 1);
    arbe_project_image_pub = nh.advertise<sensor_msgs::CompressedImage>(arbe_project_image_topic, 1);

    arbe_ego_doppler_pub = nh.advertise<std_msgs::Float64>(arbe_ego_doppler_topic, 1);
    arbe_ego_doppler_kf_pub = nh.advertise<std_msgs::Float64>(arbe_ego_doppler_kf_topic, 1);
    arbe_ego_doppler_x_pub = nh.advertise<std_msgs::Float64>(arbe_ego_doppler_x_topic, 1);
    arbe_ego_doppler_y_pub = nh.advertise<std_msgs::Float64>(arbe_ego_doppler_y_topic, 1);

    arbe_object_marker_clear.header = cloud_header;
    arbe_object_marker_clear.ns = "arbe_object";
    // arbe_object_marker.id = label;
    arbe_object_marker_clear.type = visualization_msgs::Marker::CUBE;
    arbe_object_marker_clear.action = visualization_msgs::Marker::DELETEALL;

    inti_ego_doppler_kf();

    nh.getParam("max_segment_distance", MAX_SEGMENT_DISTANCE);
    nh.getParam("min_segment_distance", MIN_SEGMENT_DISTANCE);
    nh.getParam("min_segment_number", MIN_SEGMENT_NUMBER);
    nh.getParam("max_segment_doppler_diff", MAX_SEGMENT_DOPPLER_THRE);
    nh.getParam("search_radius_rate", SEARCH_RAIUS_RATE);
    nh.getParam("static_moving_thre", STATIC_MOVING_THRE);

    nh.getParam("kf_q", Q);
    nh.getParam("kf_r", R);

    arbe_project_image_msg.header = std_msgs::Header();
    arbe_project_image_msg.header.seq = 0;
    arbe_project_image_msg.header.frame_id = "reprojection";
    arbe_project_image_msg.format = "rgb8; jpeg compressed bgr8";

    vehicle_yaw = 0;

    allocate_memory();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))//Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
    cout << "init!" << endl;
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

    A = 1;
    H = 1;

    ego_doppler_init_flag = false;
  }

  void allocate_memory() {
    arbe_origin_pcl.reset(new PointCloudA());
    arbe_segment_pcl.reset(new PointCloudA());
    arbe_outlier_pcl.reset(new PointCloudA());
    arbe_static_pcl.reset(new PointCloudA());
    arbe_static_front_pcl.reset(new PointCloudA());
    arbe_moving_pcl.reset(new PointCloudA());
    arbe_moving_front_pcl.reset(new PointCloudA());
    arbe_object_pcl.reset(new PointCloudAO());
    arbe_os_cfar_pcl.reset(new PointCloudA());

    arbe_polar_pcl.reset(new PointCloudI);

    kdtree_origin.reset(new pcl::KdTreeFLANN<PointA>());
    kdtree_polar.reset(new pcl::KdTreeFLANN<PointI>());

    arbe_project_image = cv::Mat(ROW, COL, CV_8UC3, cv::Vec3b(255, 255, 255));

  }

  void dbw_callback(const arbe_slam::CANROS::ConstPtr& dbw_msg)
  {
    if (dbw_msg->arbitration_id == 0x65)
    {
      int data[] = { dbw_msg->data[1],dbw_msg->data[0],dbw_msg->data[3],dbw_msg->data[2],dbw_msg->data[4],dbw_msg->data[5],dbw_msg->data[6] };

      // SteeringReport* sr = (SteeringReport*)(&dbw_msg->data);
      // cout << sr->angle * 0.1 << " " << sr->cmd * 0.1 << " " << sr->speed * 0.01 << " " << sr->torque * 0.0625 << "\\ " << endl;
      // cout << sr->angle * 0.1 << " " << sr->cmd * 0.1 << " " << sr->speed * 0.01
      //   << " " << sr->torque * 0.0625 << "\\ " << endl;
      int n1;
      if (dbw_msg->data[1] > 128)
      {
        n1 = 255 - dbw_msg->data[1];
      }
      else
      {
        n1 = dbw_msg->data[1];
      }
      cout << (n1 * 255 + dbw_msg->data[0]) * 0.1 << endl;
    }
  }

  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {
    clock_t start = clock();
    copy_pointcloud(arbe_origin_ros);

    get_polar_pointcloud();

    // os_cfar();

    calculate_ego_doppler();

    segment_pointcloud();

    fill_pointcloud();

    publish_pointcloud();

    ceres_solver_ego_doppler();

    reset_memery();
    clock_t end = clock();

    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("FeatureExtrace : total duration  %f", duration);
  }

  void copy_pointcloud(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {
    clock_t start = clock();
    cloud_header = arbe_origin_ros->header;
    cloud_header.stamp = ros::Time::now();
    pcl::fromROSMsg(*arbe_origin_ros, *arbe_origin_pcl);
    kdtree_origin->setInputCloud(arbe_origin_pcl);

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("copy_pointcloud : get %d origin ROS point, duration %f ",
      arbe_origin_ros->width, duration);
  }

  // void get_polar_pointcloud()
  // {
  //   clock_t start = clock();
  //   for (PointA point_a : arbe_origin_pcl->points)
  //   {
  //     PointI point_i;
  //     point_i.x = point_a.range_bin;
  //     point_i.y = point_a.azimuth_bin;
  //     point_i.z = point_a.elevation_bin;
  //     point_i.intensity = point_a.power_value * point_a.power_value;
  //     // cout << point_i.intensity;
  //     arbe_polar_pcl->push_back(point_i);
  //   }

  //   kdtree_polar->setInputCloud(arbe_polar_pcl);

  //   clock_t end = clock();
  //   double duration = (double)(end - start) / CLOCKS_PER_SEC;

  //   ROS_INFO("get_polar_pointcloud : duration %f ", duration);
  // }

  void get_polar_pointcloud()
  {
    clock_t start = clock();
    unordered_map<pair<int, int>, int, PairHash>  point_map;

    int max_row = 0;
    int max_col = 0;

    int max = 0;
    for (size_t i = 0;i < arbe_origin_pcl->size();i++)
    {
      PointA point = arbe_origin_pcl->points[i];

      arbe_origin_pcl->points[i].power = point.power_value;

      pair<int, int> row_col((int)point.range_bin, (int)point.azimuth_bin);
      if (row_col.first > max_row)
      {
        max_row = row_col.first;
      }
      if (row_col.second > max_col)
      {
        max_col = row_col.second;
      }

      if (point_map.find(row_col) == point_map.end())
      {
        point_map.emplace(row_col, point.power);
      }
      else {
        point_map[row_col] += point.power;
      }

      if (point_map[row_col] > max)
      {
        max = point_map[row_col];
      }
    }
    for (auto iter : point_map)
    {
      PointI point_i;
      point_i.x = iter.first.first;
      point_i.y = iter.first.second;
      point_i.z = 0;
      point_i.intensity = iter.second;
      arbe_polar_pcl->push_back(point_i);

      if (iter.first.first > ROW | iter.first.second > COL)
      {
        cout << iter.first.first << " " << iter.first.second << "\\";
      }
      else {
        int value = (int)(iter.second * 255 / max);
        cv::Scalar color = get_project_color(value);

        arbe_project_image.at<cv::Vec3b>(ROW - iter.first.first, iter.first.second)[0]
          = color[0];
        arbe_project_image.at<cv::Vec3b>(ROW - iter.first.first, iter.first.second)[1]
          = color[1];
        arbe_project_image.at<cv::Vec3b>(ROW - iter.first.first, iter.first.second)[2]
          = color[2];
      }
    }

    kdtree_polar->setInputCloud(arbe_polar_pcl);
    point_map.clear();
    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("get_polar_pointcloud : get %d point, duration %f , max_row %d, max_col %d",
      (int)arbe_polar_pcl->size(), duration, max_row, max_col);
  }


  cv::Scalar get_project_color(int index)
  {
    // int index = (dopplor - min_doppler) / dopplor_range * 255;
    int group_size = 51;
    int group_num = 5;
    int group = index / group_size;
    cv::Scalar ret = cv::Scalar(0, 0, 0);
    switch (group)
    {
    case 0:
      ret = cv::Scalar(255, 0, 255);
      ret[0] = (int)(255 - (index % group_size) * group_num);
      break;
    case 1:
      ret = cv::Scalar(0, 0, 255);
      ret[1] = (int)((index % group_size) * group_num);
      break;
    case 2:
      ret = cv::Scalar(0, 255, 255);
      ret[2] = (int)(255 - (index % group_size) * group_num);
      break;
    case 3:
      ret = cv::Scalar(0, 255, 0);
      ret[0] = (int)((index % group_size) * group_num);
      break;
    case 4:
      ret = cv::Scalar(255, 255, 0);
      ret[1] = (int)((255 - index % group_size) * group_num);
      break;
    case 5:
      ret = cv::Scalar(255, 255, 255);
      break;
    default:
      break;
    }
    // cout << dopplor << "\t" << min_doppler << "\t" << dopplor_range << "\t" << index << "\t" << ret << endl;
    return ret;
  }

  void os_cfar()
  {
    clock_t start = clock();
    vector<int> os_cfar_target_ind;
    for (size_t i = 0;i < arbe_polar_pcl->size();i++)
    {
      PointI point_center = arbe_polar_pcl->points[i];
      std::vector<int> point_search_ind;
      std::vector<float> point_search_sq_dis;
      int kdtree_search_number = kdtree_polar->radiusSearch(point_center, OS_CFAR_RADIUS,
        point_search_ind, point_search_sq_dis);
      // cout << "os" << i << " " << kdtree_search_number << endl;

      // if (kdtree_search_number > MIN_OS_CFAR_NUMBER)
      // {
      //   float os_cfar_u = get_kth_thre(point_search_ind, 0, point_search_ind.size() - 1,
      //     point_search_ind.size() - MIN_OS_CFAR_NUMBER);
      //   cout << os_cfar_u << " ";
      //   if (point_center.intensity > os_cfar_u * OS_CFAR_A)
      //   {
      //     os_cfar_target_ind.push_back(i);
      //   }
      // }
    }

    if (os_cfar_target_ind.size() > 0)
    {
      pcl::copyPointCloud(*arbe_origin_pcl, os_cfar_target_ind, *arbe_os_cfar_pcl);
    }
    os_cfar_target_ind.clear();
    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("os_cfar : origin %d points, os_cfar get %d points, duration %f",
      (int)arbe_origin_pcl->size(), (int)os_cfar_target_ind.size(), duration);
  }

  float get_kth_thre(std::vector<int>& a, int start_ind, int end_ind, int n)
  {
    // cout << start_ind << " " << end_ind << " " << n << endl;
    int mid_one = a[start_ind];//把第一个值作为支点
    int i = start_ind, j = end_ind;
    if (i == j) //递归出口之一
      return a[i];

    if (i < j) {
      while (i < j) {

        while (i < j && a[j] >= mid_one) {
          j--;
        }
        if (i < j) {
          a[i++] = a[j];
        }

        while (i < j && a[i] < mid_one) {
          i++;
        }
        if (i < j) {
          a[j--] = a[i];
        }
      }
      a[i] = mid_one;//支点归位
      //此时a[i]这个数必定处于它真正的位置上，左边的都比他小，右边的都比他大；
      int th = end_ind - i + 1;//计算下标为i的数第几大，都使用下标进行计算；

      if (th == n) {//正好找到
        return a[i];
      }
      else {
        if (th > n)//在支点右边找
          return get_kth_thre(a, i + 1, end_ind, n);
        else//在支点左边找第(n-th)大,因为右边th个数都比支点大
          return get_kth_thre(a, start_ind, i - 1, n - th);
      }
    }
    return 0;
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
    ROS_INFO("ceres_solver_ego_doppler : calculate %d points", int(arbe_static_pcl->size() / 5));

    for (int i = 0; i < int(arbe_static_pcl->size() / 5);i++)
    {
      PointA point = arbe_static_pcl->points[i * 5];
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

    std_msgs::Float64 ego_doppler_y;
    ego_doppler_y.data = motion_estimate[0];
    std_msgs::Float64 ego_doppler_x;
    ego_doppler_x.data = motion_estimate[1] * mount_y;

    arbe_ego_doppler_y_pub.publish(ego_doppler_y);
    arbe_ego_doppler_x_pub.publish(ego_doppler_x);

    ROS_INFO("ceres_solver_ego_doppler : velocity_y %f, yaw %f, duration %f s.",
      motion_estimate[0], motion_estimate[1], duration);
  }

  float calculate_ego_doppler()
  {
    clock_t start = clock();
    unordered_map<int, float, IntHash> doppler_counter;

    relate_doppler_list.clear();

    for (size_t i = 0; i < arbe_origin_pcl->size();i++)
    {
      PointA point = arbe_origin_pcl->points[i];
      float relate_doppler = point.doppler / (cos(point.azimuth) * cos(point.elevation));
      int relate_doppler_ten = (int)(10 * relate_doppler);
      relate_doppler_list.push_back(relate_doppler);
      if (point.range > MIN_CALCULATE_EGO_DOPPLER_RADIUS)
      {
        if (doppler_counter.find(relate_doppler_ten) == doppler_counter.end())
        {
          doppler_counter.emplace(relate_doppler_ten, abs(point.azimuth));
        }
        else
        {
          doppler_counter[relate_doppler_ten] += abs(point.azimuth);
        }
      }
    }

    PointA point_0 = arbe_origin_pcl->points[0];
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

    float ego_doppler_static = (float)ego_doppler_ten / 10.;

    std_msgs::Float64 ego_doppler_msg;
    ego_doppler_msg.data = ego_doppler_static;
    arbe_ego_doppler_pub.publish(ego_doppler_msg);

    if (ego_doppler_init_flag)
    {
      ego_doppler_new = ego_doppler_static;
      ego_doppler_kf();
    }
    else
    {
      ego_doppler = ego_doppler_old = ego_doppler_static;
      ego_doppler_init_flag = true;
    }

    doppler_counter.clear();

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("calculate_ego_doppler : ego doppler new: %f,ego doppler estimate: %f, max counter : %d, total counter : % d, duration : % f",
      ego_doppler_static, ego_doppler, max_counter, (int)arbe_origin_pcl->size(), duration);
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

    std_msgs::Float64 ego_doppler_kf_msg;
    ego_doppler_kf_msg.data = ego_doppler;
    arbe_ego_doppler_kf_pub.publish(ego_doppler_kf_msg);

    ROS_INFO("ego_doppler_kf : ego_doppler_est %F, Q %f, R %f", ego_doppler_est, Q, R);

  }

  void segment_pointcloud()
  {
    clock_t start = clock();
    moving_index_list.clear();
    for (size_t i = 0; i < arbe_origin_pcl->size();i++)
    {
      if (abs(relate_doppler_list[i] - ego_doppler) > STATIC_MOVING_THRE)
      {
        moving_index_list.push_back(i);
        // ROS_INFO("point %d, doppler diff : %f", (int)i, abs(relate_doppler_list[i] - ego_doppler));
      }
    }

    label_count = 1;
    point_label.clear();
    point_label.resize(arbe_origin_pcl->size(), 0);
    for (int index : moving_index_list)
    {
      if (point_label[index] == 0)
      {
        label_pointcloud(index);
        point_label[index] = 1;
        ROS_DEBUG("segment_pointcloud : label point %d.", index);
      }
    }

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("segment_pointcloud : origin moving points : %d, duration %f",
      (int)moving_index_list.size(), duration);
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
      ROS_DEBUG("root index : %d,BFS index %d", index, centrer_index);
      PointA point_center = arbe_origin_pcl->points[centrer_index];
      point_bfs_queue.pop();
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

  // bool get_object_point(int label, vector<int>& segmentation_index)
  // {
  //   PointCloudA::Ptr single_object_pcl(new PointCloudA());
  //   vector<float> x_list;
  //   vector<float> y_list;
  //   vector<float> z_list;
  //   vector<float> doppler_list;
  //   vector<float> power_list;

  //   int final_label = label;
  //   bool merge_flag = false;

  //   for (int i : segmentation_index)
  //   {
  //     single_object_pcl->push_back(arbe_origin_pcl->points[i]);
  //     PointA point = arbe_origin_pcl->points[i];
  //     x_list.push_back(point.x);
  //     y_list.push_back(point.y);
  //     z_list.push_back(point.z);
  //     doppler_list.push_back(point.doppler);
  //     power_list.push_back(point.power);
  //   }

  //   int arbe_moving_front_pcl_start = arbe_moving_front_pcl->size();
  //   project_pointcloud(single_object_pcl, arbe_moving_front_pcl);
  //   int arbe_moving_front_pcl_end = arbe_moving_front_pcl->size();

  //   // for (size_t i = arbe_moving_front_pcl_start; i < arbe_moving_front_pcl_end;i++)
  //   // {
  //   //   PointA point = arbe_moving_front_pcl->points[i];
  //   //   x_list.push_back(point.x);
  //   //   y_list.push_back(point.y);
  //   //   z_list.push_back(point.z);
  //   //   doppler_list.push_back(point.doppler);
  //   //   power_list.push_back(point.power);
  //   // }

  //   PointAO arbe_object = calculate_arbe_object(x_list, y_list, z_list, doppler_list, power_list, label);


  //   int merge_result = merge_arbe_object(arbe_object);
  //   if (merge_result == -1)
  //   {
  //     arbe_object_pcl->push_back(arbe_object);
  //   }
  //   else
  //   {
  //     final_label = merge_result;
  //     merge_flag = true;
  //   }

  //   ROS_DEBUG("object %d -> %d : pointcloud number : %d,  pointcloud front number : %d!", label, final_label,
  //     (int)single_object_pcl->size(), arbe_moving_front_pcl_end - arbe_moving_front_pcl_start);

  //   for (int i = arbe_moving_front_pcl_start;i < arbe_moving_front_pcl_end;i++)
  //   {
  //     arbe_moving_front_pcl->points[i].power_value = final_label;
  //   }

  //   for (PointA point : single_object_pcl->points)
  //   {
  //     point.power_value = final_label;
  //     arbe_moving_pcl->push_back(point);
  //   }
  //   return merge_flag;
  // }

  bool get_object_point(int label, vector<int>& segmentation_index)
  {
    PointCloudA::Ptr single_object_pcl(new PointCloudA());
    vector<float> x_list;
    vector<float> y_list;
    vector<float> z_list;
    vector<float> doppler_list;
    vector<float> power_list;

    int final_label = label;
    bool merge_flag = false;

    for (int i : segmentation_index)
    {
      single_object_pcl->push_back(arbe_origin_pcl->points[i]);
      PointA point = arbe_origin_pcl->points[i];
      x_list.push_back(point.x);
      y_list.push_back(point.y);
      z_list.push_back(point.z);
      doppler_list.push_back(point.doppler);
      power_list.push_back(point.power);
    }

    // get_obb(single_object_pcl);


    int arbe_moving_front_pcl_start = arbe_moving_front_pcl->size();
    project_pointcloud(single_object_pcl, arbe_moving_front_pcl);
    int arbe_moving_front_pcl_end = arbe_moving_front_pcl->size();

    // for (size_t i = arbe_moving_front_pcl_start; i < arbe_moving_front_pcl_end;i++)
    // {
    //   PointA point = arbe_moving_front_pcl->points[i];
    //   x_list.push_back(point.x);
    //   y_list.push_back(point.y);
    //   z_list.push_back(point.z);
    //   doppler_list.push_back(point.doppler);
    //   power_list.push_back(point.power);
    // }

    PointAO arbe_object = calculate_arbe_object(x_list, y_list, z_list, doppler_list, power_list, label);
    cout << arbe_object.x << "\t" << arbe_object.y << "\t" << arbe_object.z << endl;

    int merge_result = merge_arbe_object(arbe_object);
    if (merge_result == -1)
    {
      arbe_object_pcl->push_back(arbe_object);
    }
    else
    {
      final_label = merge_result;
      merge_flag = true;
    }

    ROS_DEBUG("object %d -> %d : pointcloud number : %d,  pointcloud front number : %d!", label, final_label,
      (int)single_object_pcl->size(), arbe_moving_front_pcl_end - arbe_moving_front_pcl_start);

    for (int i = arbe_moving_front_pcl_start;i < arbe_moving_front_pcl_end;i++)
    {
      arbe_moving_front_pcl->points[i].power_value = final_label;
    }

    for (PointA point : single_object_pcl->points)
    {
      point.power_value = final_label;
      arbe_moving_pcl->push_back(point);
    }
    return merge_flag;
  }


  PointAO calculate_arbe_object(vector<float> x_list, vector<float> y_list, vector<float> z_list,
    vector<float> doppler_list, vector<float> power_list, int label)
  {
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

    return arbe_object;
  }

  int merge_arbe_object(PointAO arbe_object)
  {
    int final_label = -1;
    for (size_t i = 0;i < arbe_object_pcl->size(); i++)
    {
      PointAO arbe_object_search = arbe_object_pcl->points[i];
      if ((abs(arbe_object_search.x - arbe_object.x) - abs(arbe_object_search.width + arbe_object.width) / 2 < MAX_OBJECT_DISTANCE
        && abs(arbe_object_search.y - arbe_object.y) - abs(arbe_object_search.length + arbe_object.length) / 2 < MAX_OBJECT_DISTANCE
        && abs(arbe_object_search.z - arbe_object.z) - abs(arbe_object_search.high + arbe_object.high) / 2 < MAX_OBJECT_DISTANCE
        && abs(arbe_object_search.doppler - arbe_object.doppler) < Min_OBJECT_DOPPLER)
        | (abs(arbe_object_search.x - arbe_object.x) - abs(arbe_object_search.width + arbe_object.width) / 2 <= 0
          && abs(arbe_object_search.y - arbe_object.y) - abs(arbe_object_search.length + arbe_object.length) / 2 <= 0
          && abs(arbe_object_search.z - arbe_object.z) - abs(arbe_object_search.high + arbe_object.high) / 2 <= 0
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

        final_label = i;

        ROS_DEBUG("input label : %d, final label : %d", arbe_object.label, final_label);
        break;
      }
    }
    return final_label;
  }

  void get_obb(PointCloudA::Ptr single_object_pcl)
  {
    ROS_INFO("start_odd");
    PointCloudT::Ptr single_object_xyz_pcl(new PointCloudT);
    pcl::copyPointCloud(*single_object_pcl, *single_object_xyz_pcl);
    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    feature_extractor.setInputCloud(single_object_xyz_pcl);
    feature_extractor.compute();
    ROS_INFO("comput odd");
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);
    ROS_INFO("get_odd");

    cout << position_OBB.x << "\t" << position_OBB.y << "\t" << position_OBB.z << endl;
    cout << major_value << "\t" << middle_value << "\t" << minor_value << endl;
  }

  // void get_obb(PointCloudA::Ptr single_object_pcl)
  // {

  //   pcl::MomentOfInertiaEstimation <PointA> feature_extractor;
  //   feature_extractor.setInputCloud(single_object_pcl);
  //   feature_extractor.compute();

  //   std::vector <float> moment_of_inertia;
  //   std::vector <float> eccentricity;
  //   PointA min_point_AABB;
  //   PointA max_point_AABB;
  //   PointA min_point_OBB;
  //   PointA max_point_OBB;
  //   PointA position_OBB;
  //   Eigen::Matrix3f rotational_matrix_OBB;
  //   float major_value, middle_value, minor_value;
  //   Eigen::Vector3f major_vector, middle_vector, minor_vector;
  //   Eigen::Vector3f mass_center;

  //   feature_extractor.getMomentOfInertia(moment_of_inertia);
  //   feature_extractor.getEccentricity(eccentricity);
  //   feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  //   feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  //   feature_extractor.getEigenValues(major_value, middle_value, minor_value);
  //   feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  //   feature_extractor.getMassCenter(mass_center);

  //   cout << position_OBB.x << "\t" << position_OBB.y << "\t" << position_OBB.z << endl;
  //   cout << major_value << "\t" << middle_value << "\t" << minor_value << endl;
  // }

  void publish_pointcloud()
  {
    clock_t start = clock();

    ROS_INFO("publish_pointcloud : segment %d segment point into %d objects, get %d outlier!",
      (int)arbe_moving_pcl->size(), (int)label_count, (int)arbe_outlier_pcl->size());

    sensor_msgs::PointCloud2 arbe_moving_ros;
    pcl::toROSMsg(*arbe_moving_pcl, arbe_moving_ros);
    arbe_moving_ros.header = cloud_header;
    arbe_moving_pub.publish(arbe_moving_ros);

    sensor_msgs::PointCloud2 arbe_moving_front_ros;
    pcl::toROSMsg(*arbe_moving_front_pcl, arbe_moving_front_ros);
    arbe_moving_front_ros.header = cloud_header;
    arbe_moving_front_pub.publish(arbe_moving_front_ros);

    sensor_msgs::PointCloud2 arbe_outlier_ros;
    pcl::toROSMsg(*arbe_outlier_pcl, arbe_outlier_ros);
    arbe_outlier_ros.header = cloud_header;
    arbe_outlier_pub.publish(arbe_outlier_ros);

    sensor_msgs::PointCloud2 arbe_static_ros;
    pcl::toROSMsg(*arbe_static_pcl, arbe_static_ros);
    arbe_static_ros.header = cloud_header;
    arbe_static_pub.publish(arbe_static_ros);

    sensor_msgs::PointCloud2 arbe_static_front_ros;
    pcl::toROSMsg(*arbe_static_front_pcl, arbe_static_front_ros);
    arbe_static_front_ros.header = cloud_header;
    arbe_static_front_pub.publish(arbe_static_front_ros);

    sensor_msgs::PointCloud2 arbe_object_ros;
    pcl::toROSMsg(*arbe_object_pcl, arbe_object_ros);
    arbe_object_ros.header = cloud_header;
    arbe_object_pub.publish(arbe_object_ros);

    sensor_msgs::PointCloud2 arbe_os_cfar_ros;
    pcl::toROSMsg(*arbe_os_cfar_pcl, arbe_os_cfar_ros);
    arbe_os_cfar_ros.header = cloud_header;
    arbe_os_cfar_pub.publish(arbe_os_cfar_ros);


    arbe_project_image_msg.header.stamp = ros::Time::now();
    arbe_project_image_msg.header.seq += 1;
    cv::imencode(".jpg", arbe_project_image, arbe_project_image_msg.data);
    arbe_project_image_pub.publish(arbe_project_image_msg);

    clock_t end = clock();
    float duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("publish_pointcloud : duration %f", duration);
  }

  void reset_memery()
  {
    arbe_object_marker_msg.markers.clear();
    arbe_object_marker_msg.markers.push_back(arbe_object_marker_clear);
    arbe_object_pcl->clear();
    arbe_segment_pcl->clear();
    arbe_outlier_pcl->clear();
    arbe_moving_pcl->clear();
    arbe_static_pcl->clear();
    arbe_moving_front_pcl->clear();
    arbe_static_front_pcl->clear();
    arbe_polar_pcl->clear();

    arbe_project_image = cv::Mat(ROW, COL, CV_8UC3, cv::Vec3b(255, 255, 255));

  }

  void get_arbe_object_marker()
  {
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
  }

  void project_pointcloud(PointCloudA::Ptr source_pcl, PointCloudA::Ptr target_pcl)
  {
    unordered_map<pair<int, int>, pair<int, float>, PairHash>  point_map;
    for (size_t i = 0;i < source_pcl->size();i++)
    {
      PointA point = source_pcl->points[i];
      pair<int, int> row_col((int)point.azimuth_bin, (int)point.elevation_bin);

      if (point_map.find(row_col) == point_map.end())
      {
        // pair<int, float> point_info(i, point.range);
        pair<int, float> point_info(i, point.power);
        point_map.emplace(row_col, point_info);
      }
      else {
        // if (point.range < point_map[row_col].second)
        if (point.power < point_map[row_col].second)
        {
          // pair<int, float> point_info(i, point.range);
          pair<int, float> point_info(i, point.power);
          point_map.emplace(row_col, point_info);
        }
      }
    }
    for (auto iter : point_map)
    {
      target_pcl->push_back(source_pcl->points[iter.second.first]);
    }
  }

  void fill_pointcloud()
  {
    unordered_map<pair<int, int>, pair<int, float>, PairHash>  static_point_map;

    for (int i = 0;i < arbe_origin_pcl->size();i++)
    {
      int label = point_label[i];
      if (label == OUTLIER_LABEL)
      {
        arbe_outlier_pcl->push_back(arbe_origin_pcl->points[i]);
      }
      else if (label != 0)
      {
        arbe_moving_pcl->push_back(arbe_origin_pcl->points[i]);
        arbe_moving_pcl->back().power_value = label;
        arbe_moving_pcl->back().doppler_bin = relate_doppler_list[i];
      }
      else
      {
        arbe_static_pcl->push_back(arbe_origin_pcl->points[i]);
        arbe_static_pcl->back().power = relate_doppler_list[i];
      }
    }

    project_pointcloud(arbe_static_pcl, arbe_static_front_pcl);
    get_arbe_object_marker();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_extrace");

  FeatureExtrace fe;

  ros::spin();
}
