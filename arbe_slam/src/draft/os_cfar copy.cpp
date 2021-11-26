#include "os_cfar.h"

class OSCFAR
{
private:
  vector<cv::Vec3b> color_list;
  unordered_map<pair<int, int>, ProjectInfo, PairHash> project_map;
  unordered_map<pair<int, int>, int, PairHash> doppler_map;

public:
  OSCFAR()
  {
    get_color_list();
    cout << "OSCFAR init" << endl;
  }

  void run(
    PointCloudA::Ptr arbe_origin_pcl,
    unordered_map<pair<int, int>, vector<int>, PairHash>& point_map,
    cv::Mat& arbe_project_image,
    PointCloudA::Ptr arbe_os_cfar_pcl)
  {
    clock_t start = clock();
    int max_power = 0;
    // cout << "run" << endl;
    get_project_map(arbe_origin_pcl, point_map, max_power);
    // cout << "get_project_map" << endl;
    get_project_image(point_map, max_power, arbe_project_image);
    // cout << "get_project_image" << endl;
    os_cfar(arbe_origin_pcl, point_map, max_power, arbe_project_image, arbe_os_cfar_pcl);
    // cout << " os_cfar" << endl;

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("get_project_pointcloud : get %d point, duration %f", (int)point_map.size(), duration);
  }

  void get_project_map(
    PointCloudA::Ptr arbe_origin_pcl,
    unordered_map<pair<int, int>, vector<int>, PairHash>& point_map,
    int& max_power)
  {
    point_map.clear();
    project_map.clear();

    int max_row = 0;
    int max_col = 0;

    for (size_t i = 0;i < arbe_origin_pcl->size();i++)
    {
      PointA point = arbe_origin_pcl->points[i];
      // pair<int, int> row_col((int)point.elevation_bin, (int)point.azimuth_bin);
      pair<int, int> row_col(16, (int)point.azimuth_bin);
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
        point_map.emplace(row_col, vector<int>({ (int)point.power,(int)i }));

        ProjectInfo pi;
        pi.sum_power += point.power;
        pi.index.push_back(i);
        project_map.emplace(row_col, pi);
      }
      else
      {
        // if (point.power > point_map[row_col][0])
        // {
        //   point_map[row_col][0] = point.power;
        // }
        point_map[row_col][0] += point.power;
        point_map[row_col].push_back((int)i);

        project_map[row_col].sum_power += point.power;
        project_map[row_col].index.push_back(i);
      }

      if (point_map[row_col][0] > max_power)
      {
        max_power = point_map[row_col][0];
      }
    }
    // ROS_INFO("get_project_map : get %d points", (int)point_map.size());
  }

  float get_shang(int& number)
  {
    float shang;
    for (auto iter : doppler_map)
    {
      float p = (float)iter.second / (float)number;
      shang -= p * log2(p);
    }
    return shang;
  }

  void get_project_image(
    unordered_map<pair<int, int>, vector<int>, PairHash>& point_map,
    int& max_power,
    cv::Mat& arbe_project_image)
  {
    for (auto iter : point_map)
    {
      doppler_map.clear();
      get_doppler_map(iter.second);
      int len = iter.second.size() - 1;
      int shang = get_shang(len);

      iter.second[0] = shang;
      if (shang > max_power)
      {
        max_power = shang;
      }

      // ROS_INFO("get_project_image : %d , %d ", iter.first.first, iter.first.second);
    }

    for (auto iter : point_map)
    {
      if (iter.first.first > ROW | iter.first.second > COL)
      {
        cout << iter.first.first << " " << iter.first.second << "\\";
      }
      else
      {
        int value = (int)(iter.second[0] * 255 / max_power);

        draw_image(ROW - iter.first.first, COL - iter.first.second, value, arbe_project_image);
      }
    }
  }

  // ROS_INFO("get_project_image : get %d * %d", ROW, COL);


  void get_doppler_map(vector<int>& data)
  {
    for (int i = 1;i < data.size();i++)
    {
      int doppler_bin_group = (int)arbe_origin_pcl->points[data[i]].doppler_bin / DOPPELR_GROUP_WIDTH;
      int range_bin_group = (int)arbe_origin_pcl->points[data[i]].range_bin / RANGE_GROUP_WIDTH;
      int elevation_bin_group = (int)arbe_origin_pcl->points[data[i]].elevation_bin / ELEVATION_GROUP_WIDTH;

      int range_group = elevation_bin_group * 32 + doppler_bin_group;
      pair<int, int> range_doppler(range_group, elevation_bin_group);

      if (doppler_map.find(range_doppler) == doppler_map.end())
      {
        doppler_map.emplace(range_doppler, 1);
      }
      else
      {
        doppler_map[range_doppler] += 1;
      }
    }
  }

  int get_std(vector<int>& input_list)
  {
    float mean, sum;
    mean = sum = 0;
    float len = (float)input_list.size();
    for (int i : input_list)
    {
      sum += i;
    }
    mean = sum / len;

    float variance, variance_sum;
    variance = variance_sum = 0;

    for (int i : input_list)
    {
      variance_sum += (i - mean) * (i - mean);
    }
    variance = variance_sum / len;

    return (int)sqrt(variance);
  }

  void os_cfar(
    PointCloudA::Ptr arbe_origin_pcl,
    unordered_map<pair<int, int>, vector<int>, PairHash>& point_map,
    int& max_power,
    cv::Mat& arbe_project_image,
    PointCloudA::Ptr arbe_os_cfar_pcl)
  {
    clock_t start = clock();
    vector<int> os_cfar_target_ind;

    for (auto iter : point_map)
    {
      pair<int, int> i_j = iter.first;
      int value = (int)(point_map[i_j][0] * 255 / max_power);

      // if (iter.second.size() > MAX_ELEVATION_NUMBER
      //   && *max_element(iter.second.begin() + 1, iter.second.end())
      //   - *min_element(iter.second.begin() + 1, iter.second.end()) > MAX_ELEVATION_RANGE)
      // if (point_map[i_j][0] > MAX_SHANG)
      // {
      //   draw_image(ROW - i_j.first, COL - i_j.second + 2 * COL, value, arbe_project_image);
      // }
      // else
      // {
      vector<int> power_list;
      get_surround_power(i_j, point_map, power_list);
      float os_cfar_u;
      get_threshould(point_map, power_list, i_j, os_cfar_u);
      // cout << os_cfar_u << " " << point_map[i_j][0] << "\\";
      // | point_map[i_j][0] < MIN_SHANG
      if (point_map[i_j][0] < os_cfar_u * OS_CFAR_A)
      {
        int len = point_map[i_j].size();
        for (int ind = 1;ind < len; ind++)
        {
          os_cfar_target_ind.push_back(point_map[i_j][ind]);
        }
        draw_image(ROW - i_j.first, COL - i_j.second + COL, value, arbe_project_image);
      }
      else
      {
        draw_image(ROW - i_j.first, COL - i_j.second + 2 * COL, value, arbe_project_image);
        cout << os_cfar_u << " " << point_map[i_j][0] << "\\";
        // }
      }
    }
    cout << endl;

    if (os_cfar_target_ind.size() > 0)
    {
      // pcl::copyPointCloud(*arbe_origin_pcl, os_cfar_target_ind, *arbe_os_cfar_pcl);
      for (int index : os_cfar_target_ind)
      {
        arbe_os_cfar_pcl->push_back(arbe_origin_pcl->points[index]);
        // cout << index << "\\";
      }
      os_cfar_target_ind.clear();
    }

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;

    ROS_INFO("os_cfar (%d, %d, %d, %d, %f, %d, %d) : origin %d points, os_cfar get %d points, duration %f",
      OS_CFAR_RADIUS_RANGE, OS_CFAR_RADIUS_AZIMUTH, MIN_OS_CFAR_NUMBER, MAX_OS_CFAR_NUMBER,
      OS_CFAR_A, MAX_ELEVATION_NUMBER, MAX_ELEVATION_RANGE,
      (int)arbe_origin_pcl->size(), (int)arbe_os_cfar_pcl->size(), duration);

  }

  // void draw_image(int row, int col, int value, cv::Mat& arbe_project_image)
  // {
  //   if (row < ROW && col < COL * 3)
  //   {
  //     arbe_project_image.at<cv::Vec3b>(row, col) = color_list[value];
  //     // arbe_project_image.at<cv::Vec3b>(row, col) = cv::Vec3b(value, value, value);
  //   }
  // }

  void draw_image(int row, int col, int value, cv::Mat& arbe_project_image)
  {
    if (row < ROW && col < COL * 3)
    {
      int high = value * ROW / 255;
      for (int i = 0; i < high;i++)
      {
        arbe_project_image.at<cv::Vec3b>(ROW - i - 1, col) = color_list[value];
      }

      // arbe_project_image.at<cv::Vec3b>(row, col) = cv::Vec3b(value, value, value);
    }
  }


  void get_surround_power(
    pair<int, int>& i_j,
    unordered_map<pair<int, int>, vector<int>, PairHash>& point_map,
    vector<int>& power_list)
  {
    int i = i_j.first;
    int j = i_j.second;
    // for (int row = i - OS_CFAR_RADIUS_RANGE;row <= i + OS_CFAR_RADIUS_RANGE;row++)
    ROS_DEBUG("%d %d  ", i, j);
    for (int row = i;row < i + 1;row++)
    {
      for (int col = j - OS_CFAR_RADIUS_AZIMUTH;col <= j + OS_CFAR_RADIUS_AZIMUTH;col++)
      {
        ROS_DEBUG("%d %d %d %d ", i, j, row, col);
        pair<int, int> row_col(row, col);
        if (point_map.find(row_col) != point_map.end())
        {
          // if (row != i && col != j)
          if (col != j)
          {
            power_list.push_back(point_map[row_col][0]);
          }
        }
      }
    }
  }

  void get_threshould(
    unordered_map<pair<int, int>, vector<int>, PairHash>& point_map,
    vector<int>& power_list,
    pair<int, int>& i_j,
    float& os_cfar_u)
  {
    // cout << "len = " << power_list.size() << " ";
    if (power_list.size() > MIN_OS_CFAR_NUMBER)
    {
      // os_cfar_u = get_kth_thre(power_list, 0, power_list.size() - 1, MIN_OS_CFAR_NUMBER);
      os_cfar_u = get_kth_thre(power_list, 0, power_list.size() - 1, MIN_OS_CFAR_NUMBER);
      // ROS_INFO("power_list : %d; os_cfar_u %f : power : %d ", (int)power_list.size(), os_cfar_u, point_map[i_j][0]);
      power_list.clear();
    }
    else
    {
      os_cfar_u = point_map[i_j][0];
    }
  }

  float get_kth_thre(
    std::vector<int>& a,
    int start_ind,
    int end_ind,
    int n)
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

  void get_color_list()
  {
    for (int i = 0;i < 256;i++)
    {
      color_list.push_back(get_project_color(i));
    }
  }

  cv::Vec3b get_project_color(int index)
  {
    // int index = (dopplor - min_doppler) / dopplor_range * 255;
    int group_size = 256;
    int group_num = 1;
    int group = index / group_size;
    cv::Vec3b ret = cv::Vec3b(255, 0, 0);
    ret[0] = 255 - index % 256;
    ret[2] = index % 256;
    return ret;
  }

  cv::Vec3b get_project_color_2(int index)
  {
    // int index = (dopplor - min_doppler) / dopplor_range * 255;
    int group_size = 51;
    int group_num = 5;
    int group = index / group_size;
    cv::Vec3b ret = cv::Vec3b(0, 0, 0);
    switch (group)
    {
    case 0:
      ret = cv::Vec3b(255, 0, 255);
      ret[0] = (int)(255 - (index % group_size) * group_num);
      break;
    case 1:
      ret = cv::Vec3b(0, 0, 255);
      ret[1] = (int)((index % group_size) * group_num);
      break;
    case 2:
      ret = cv::Vec3b(0, 255, 255);
      ret[2] = (int)(255 - (index % group_size) * group_num);
      break;
    case 3:
      ret = cv::Vec3b(0, 255, 0);
      ret[0] = (int)((index % group_size) * group_num);
      break;
    case 4:
      ret = cv::Vec3b(0, 0, 0);
      ret[2] = (int)((255 - index % group_size) * group_num);
      break;
    case 5:
      ret = cv::Vec3b(255, 255, 255);
      break;
    default:
      break;
    }
    return ret;
  }


};

OSCFAR os_cfar;

void copy_pointcloud(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
{
  clock_t start = clock();

  cloud_header = arbe_origin_ros->header;
  pcl::fromROSMsg(*arbe_origin_ros, *arbe_origin_pcl);

  clock_t end = clock();
  double duration = (double)(end - start) / CLOCKS_PER_SEC;
  ROS_INFO("copy_pointcloud : get %d origin ROS point, duration %f ",
    arbe_origin_ros->width, duration);
}

void publish_pointcloud()
{
  clock_t start = clock();

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
  arbe_os_cfar_pcl->clear();
  arbe_project_image = cv::Mat(ROW, COL * 3, CV_8UC3, cv::Vec3b(255, 255, 255));
}

void update_param(ros::NodeHandle nh)
{
  nh.getParam("os_cfar_radius_range", OS_CFAR_RADIUS_RANGE);
  nh.getParam("os_cfar_radius_azimuth", OS_CFAR_RADIUS_AZIMUTH);
  nh.getParam("min_os_cfar_number", MIN_OS_CFAR_NUMBER);
  nh.getParam("max_os_cfar_number", MAX_OS_CFAR_NUMBER);
  nh.getParam("os_cfar_a", OS_CFAR_A);
  nh.getParam("max_elevation_number", MAX_ELEVATION_NUMBER);
  nh.getParam("max_elevation_range", MAX_ELEVATION_RANGE);

  nh.getParam("max_shang", MAX_SHANG);
  nh.getParam("min_shang", MIN_SHANG);
  nh.getParam("range_group_width", RANGE_GROUP_WIDTH);
  nh.getParam("doppler_group_width", DOPPELR_GROUP_WIDTH);
  nh.getParam("elevation_group_width", ELEVATION_GROUP_WIDTH);

}

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
{
  // cout << "get ros msg!" << endl;
  copy_pointcloud(arbe_origin_ros);
  // cout << "copy_pointcloud!" << endl;
  os_cfar.run(arbe_origin_pcl, point_map, arbe_project_image, arbe_os_cfar_pcl);
  publish_pointcloud();
  reset_memery();
}

void init(ros::NodeHandle nh)
{
  arbe_origin_pcl.reset(new PointCloudA());
  arbe_os_cfar_pcl.reset(new PointCloudA());
  arbe_project_image = cv::Mat(ROW, COL * 3, CV_8UC3, cv::Vec3b(255, 255, 255));

  arbe_project_image_msg.header = std_msgs::Header();
  arbe_project_image_msg.header.seq = 0;
  arbe_project_image_msg.header.frame_id = "reprojection";
  arbe_project_image_msg.format = "rgb8; jpeg compressed bgr8";


  update_param(nh);

  cout << "init ros" << endl;
}





string int2str(int n) {

  ostringstream oss;
  oss << n;
  return oss.str();
}


void tune()
{
  // vector<string> file_list("650698691.pcd", "650698821.pcd", "650699553.pcd ",
//   "650698760.pcd", "650699164.pcd");
  // vector<string> file_list({ "1637051754_751604930.pcd", });

  vector<string> file_list({ "1637051750_281815324.pcd", });


  string path = "/home/qinguoyu/radar_slam/";
  arbe_origin_pcl.reset(new PointCloudA());
  arbe_os_cfar_pcl.reset(new PointCloudA());
  arbe_project_image = cv::Mat(ROW, COL * 3, CV_8UC3, cv::Vec3b(255, 255, 255));

  // range
  // OS_CFAR_RADIUS_AZIMUTH = 2;
  // OS_CFAR_RADIUS_RANGE = 13;
  // MIN_OS_CFAR_NUMBER = 33;

  // elevation
  OS_CFAR_RADIUS_AZIMUTH = 2;;
  OS_CFAR_RADIUS_RANGE = 15;
  MIN_OS_CFAR_NUMBER = (OS_CFAR_RADIUS_AZIMUTH * 2 + 1) * (OS_CFAR_RADIUS_RANGE * 2 + 1) / 4;

  // MIN_OS_CFAR_NUMBER = 7;
  int OS_CFAR_A_ten = 11;

  for (string name : file_list)
  {
    string filename = path + name;
    cout << "loading " << filename << endl;
    if (pcl::io::loadPCDFile<PointA>(filename, *arbe_origin_pcl) != -1)
    {
      cout << "pointcloud width  " << arbe_origin_pcl->size() << endl;
      for (RANGE_GROUP_WIDTH = 1;RANGE_GROUP_WIDTH < 20; RANGE_GROUP_WIDTH++)
      {
        for (DOPPELR_GROUP_WIDTH = 3;DOPPELR_GROUP_WIDTH < 4;DOPPELR_GROUP_WIDTH++)
        {
          // for (int OS_CFAR_A_ten = 11;OS_CFAR_A_ten < 30; OS_CFAR_A_ten++)
          // {
          string image_name = path + "1637051750_"
            + int2str(OS_CFAR_RADIUS_AZIMUTH) + "_"
            + int2str(OS_CFAR_RADIUS_RANGE) + "_"
            + int2str(RANGE_GROUP_WIDTH) + "_"
            + int2str(DOPPELR_GROUP_WIDTH) + ".jpg";
          cout << image_name << endl;

          MIN_OS_CFAR_NUMBER = (OS_CFAR_RADIUS_AZIMUTH * 2 + 1) * (OS_CFAR_RADIUS_RANGE * 2 + 1) / 4;

          OS_CFAR_A = (float)OS_CFAR_A_ten / 10.;
          os_cfar.run(arbe_origin_pcl, point_map, arbe_project_image, arbe_os_cfar_pcl);

          cv::imwrite(image_name, arbe_project_image);
          reset_memery();
          // }
        }
      }
    }
    else
    {
      cout << "load " << filename << " false!" << endl;
    }
  }
}

int main(int argc, char** argv)
{
  for (int i = 0;i < argc;i++)
  {
    cout << i << " " << argv[i] << endl;
  }
  if (argc > 1)
  {
    if (*argv[1] == '1')
    {
      tune();
      return 0;
    }
  }

  ros::init(argc, argv, "os_cfar");
  ros::NodeHandle nh("~");
  init(nh);
  ros::Subscriber arbe_origin_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_origin_topic, 1,
    pointcloud_callback);
  arbe_os_cfar_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_os_cfar_topic, 1);
  arbe_project_image_pub = nh.advertise<sensor_msgs::CompressedImage>(arbe_project_image_topic, 1);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    update_param(nh);
    // ROS_INFO("SET PARAM: %f", OS_CFAR_A);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;

}
