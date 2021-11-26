#include "multipath.h"


class MutiPath
{
private:
  ros::NodeHandle nh;

  ros::Subscriber arbe_origin_sub;
  ros::Publisher arbe_mutipath_pub;
  ros::Publisher arbe_project_image_pub;

  std_msgs::Header  cloud_header;
  PointCloudA::Ptr arbe_origin_pcl;
  PointCloudA::Ptr arbe_mutipath_pcl;

  cv::Mat arbe_project_image;
  cv::Mat arbe_project_value;

  cv::Mat arbe_project_gray;
  cv::Mat arbe_project_binary;
  cv::Mat arbe_project_bgr;
  cv::Mat arbe_project_close;
  // cv::Mat arbe_project_open;
  // cv::Mat arbe_project_filted;
  // cv::Mat arbe_project_canny;
  // cv::Mat arbe_project_hough;
  cv::Mat arbe_project_detect;
  cv::Mat arbe_project_fit;
  cv::Mat arbe_project_output;

  sensor_msgs::CompressedImage arbe_project_image_msg;


public:

  MutiPath() : nh("~")
  {
    arbe_origin_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_origin_topic, 1,
      &MutiPath::pointcloud_callback, this);

    arbe_mutipath_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_mutipath_topic, 1);
    arbe_project_image_pub = nh.advertise<sensor_msgs::CompressedImage>(arbe_project_image_topic, 1);

    arbe_project_image_msg.header = std_msgs::Header();
    arbe_project_image_msg.header.seq = 0;
    arbe_project_image_msg.header.frame_id = "reprojection";
    arbe_project_image_msg.format = "rgb8; jpeg compressed bgr8";

    allocate_memory();

    // tune();
  }

  void allocate_memory()
  {
    arbe_origin_pcl.reset(new PointCloudA());
    arbe_project_gray = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    arbe_project_value = cv::Mat(ROW, COL, CV_16UC1, cv::Scalar(0));
    cout << "init" << endl;
  }

  void tune()
  {
    string name = "1637300848_2523142.pcd";
    string path = "/home/qinguoyu/radar_slam/";

    string filename = path + name;

    if (pcl::io::loadPCDFile<PointA>(filename, *arbe_origin_pcl) != -1)
    {
      get_project_image();
      cv::threshold(arbe_project_gray, arbe_project_binary, 25, 255, cv::THRESH_BINARY);
      // do_morphologyex(arbe_project_binary, arbe_project_close, 3);
      find_fence(arbe_project_binary);

      // do_median_blur(arbe_project_close, arbe_project_filted);
      // do_canny(arbe_project_filted, arbe_project_canny);

      // cv::Mat arbe_project_canny_bgr;
      // cv::cvtColor(arbe_project_canny, arbe_project_canny_bgr, cv::COLOR_GRAY2BGR);

      // for (int factor = 1;factor < 10;factor += 1)
      // {
      //   do_hough(arbe_project_canny, arbe_project_hough, factor);

      //   cv::Mat merge;
      //   cv::hconcat(arbe_project_canny_bgr, arbe_project_hough, merge);

      //   string image_name = path
      //     + int2str(factor) + ".jpg";
      //   cv::imwrite(image_name, merge);
      // }
    }
  }

  void get_hist_nozero(cv::Mat& src, pair<int, int>& left_right_index, vector<pair<int, int>>& no_zero_index)
  {
    int hist[COL] = { 0 };
    for (int row = 0; row < ROW;row++)
    {
      for (int col = 0;col < COL;col++)
      {
        if ((int)src.at < uchar >(row, col) == 255)
        {
          if (row > ROW / 2)
            hist[col] += 1;
          no_zero_index.push_back(pair<int, int>(row, col));
        }
      }
    }
    int* left = max_element(hist, hist + COL / 2);
    int* right = max_element(hist + COL / 2, hist + COL - 1);

    left_right_index = pair<int, int>(left - hist, right - hist);
    // cout << left - hist << " : " << *left << " " << right - hist << " : " << *right << endl;
  }

  void get_fence_position(
    cv::Mat& src,
    pair<int, int>& left_right_index,
    vector<pair<int, int>>& no_zero_index,
    vector<pair<int, int>>& left_point,
    vector<pair<int, int>>& right_point)
  {
    // int left_col = left_right_index.first;
    // int right_col = left_right_index.second;
    // int iter_row = ROW - FENCE_WINDOW_HIGH;
    // int left_gap_counter = 0;
    // for (int window = 0;window < FENCE_WINDOW_NUMBER;window++)
    // {

    //   vector<pair<int, int>> left_index;
    //   for (auto iter : no_zero_index)
    //   {
    //     if (abs(iter.first - iter_row) <= FENCE_WINDOW_HIGH
    //       && abs(iter.second - left_col) <= FENCE_WINDOW_WIDTH)
    //     {
    //       left_index.push_back(iter);
    //     }
    //   }

    //   if (left_index.size() > MIN_PIXEL)
    //   {
    //     int sum = 0;
    //     for (auto iter : left_index)
    //     {
    //       sum += iter.second;
    //     }
    //     left_col = sum / left_index.size();
    //     left_point.push_back(pair<int, int>(iter_row, left_col));

    //     left_gap_counter = 0;
    //   }
    //   else
    //   {
    //     if (left_point.size() > 1)
    //     {
    //       left_gap_counter += 1;
    //       if (left_gap_counter > MAX_GAP)
    //       {
    //         break;
    //       }
    //     }
    //   }
    //   iter_row -= 2 * FENCE_WINDOW_HIGH;
    // }

    // int right_col = left_right_index.second;
    // iter_row = ROW - FENCE_WINDOW_HIGH;
    // int right_gap_counter = 0;
    // for (int window = 0;window < FENCE_WINDOW_NUMBER;window++)
    // {

    //   vector<pair<int, int>> right_index;
    //   for (auto iter : no_zero_index)
    //   {
    //     if (abs(iter.first - iter_row) <= FENCE_WINDOW_HIGH
    //       && abs(iter.second - right_col) <= FENCE_WINDOW_WIDTH)
    //     {
    //       right_index.push_back(iter);
    //     }
    //   }

    //   if (right_index.size() > MIN_PIXEL)
    //   {
    //     int sum = 0;
    //     for (auto iter : right_index)
    //     {
    //       sum += iter.second;
    //     }
    //     right_col = sum / right_index.size();
    //     right_point.push_back(pair<int, int>(iter_row, right_col));
    //     right_gap_counter = 0;
    //   }
    //   else
    //   {
    //     if (right_point.size() > 1)
    //     {
    //       right_gap_counter += 1;
    //       if (left_gap_counter > MAX_GAP)
    //       {
    //         break;
    //       }
    //     }
    //   }

    //   iter_row -= 2 * FENCE_WINDOW_HIGH;
    // }

    find_line(left_right_index.first, no_zero_index, left_point);
    find_line(left_right_index.second, no_zero_index, right_point);

    ROS_INFO("get_fence_position : left %d points, right %d points", (int)left_point.size(), (int)right_point.size());
  }


  void find_line(int init_col,
    vector<pair<int, int>>& no_zero_index,
    vector<pair<int, int>>& line_point)
  {
    int col = init_col;
    int row = ROW - FENCE_WINDOW_HIGH;
    int gap_counter = 0;
    for (int window = 0;window < FENCE_WINDOW_NUMBER;window++)
    {

      vector<pair<int, int>> index;
      for (auto iter : no_zero_index)
      {
        if (abs(iter.first - row) <= FENCE_WINDOW_HIGH
          && abs(iter.second - col) <= FENCE_WINDOW_WIDTH)
        {
          index.push_back(iter);
        }
      }

      if (index.size() > MIN_PIXEL)
      {
        int sum = 0;
        for (auto iter : index)
        {
          sum += iter.second;
        }
        col = sum / index.size();
        line_point.push_back(pair<int, int>(row, col));
        gap_counter = 0;
      }
      else
      {
        if (line_point.size() > 1)
        {
          gap_counter += 1;
          if (gap_counter > MAX_GAP)
          {
            break;
          }
        }
      }
      row -= 2 * FENCE_WINDOW_HIGH;
    }
  }

  void draw_fence_result(cv::Mat& src, vector<pair<int, int>>& left_point, vector<pair<int, int>>& right_point,
    Eigen::MatrixXd left_w, Eigen::MatrixXd right_w)
  {
    cv::Mat src_bgr;
    cv::cvtColor(src, src_bgr, cv::COLOR_GRAY2BGR);

    if (!left_w.isZero())
    {
      for (auto iter : left_point)
      {
        cv::Rect rect(iter.second - FENCE_WINDOW_WIDTH, iter.first - FENCE_WINDOW_HIGH,
          FENCE_WINDOW_WIDTH * 2, FENCE_WINDOW_HIGH * 2);
        cv::rectangle(arbe_project_detect, rect, cv::Scalar(0, 0, 255));
        cv::circle(arbe_project_detect, cv::Point(iter.second, iter.first), 5, cv::Scalar(0, 0, 255), -1);
      }

      for (int row = left_point[left_point.size() - 1].first;row < left_point[0].first;row++)
      {
        int col = int(left_w(2) * (float)pow(row, 2)
          + left_w(1) * (float)pow(row, 1)
          + left_w(0) * (float)pow(row, 0));
        if (row > 0 && row < ROW && col>1 && col < COL - 1)
        {
          arbe_project_fit.at<cv::Vec3b>(row, col - 1) = cv::Vec3b(0, 0, 255);
          arbe_project_fit.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 255);
          arbe_project_fit.at<cv::Vec3b>(row, col + 1) = cv::Vec3b(0, 0, 255);
        }
      }
    }

    if (!right_w.isZero())
    {
      for (auto iter : right_point)
      {
        cv::Rect rect(iter.second - FENCE_WINDOW_WIDTH, iter.first - FENCE_WINDOW_HIGH,
          FENCE_WINDOW_WIDTH * 2, FENCE_WINDOW_HIGH * 2);
        cv::rectangle(arbe_project_detect, rect, cv::Scalar(255, 0, 0));
        cv::circle(arbe_project_detect, cv::Point(iter.second, iter.first), 5, cv::Scalar(255, 0, 0), -1);
      }

      for (int row = right_point[right_point.size() - 1].first;row < right_point[0].first;row++)
      {
        int col = int(right_w(2) * (float)pow(row, 2)
          + right_w(1) * (float)pow(row, 1)
          + right_w(0) * (float)pow(row, 0));
        if (row > 0 && row < ROW && col>1 && col < COL - 1)
        {
          arbe_project_fit.at<cv::Vec3b>(row, col - 1) = cv::Vec3b(255, 0, 0);
          arbe_project_fit.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 0, 0);
          arbe_project_fit.at<cv::Vec3b>(row, col + 1) = cv::Vec3b(255, 0, 0);
        }
      }

    }
  }

  void fit_poly(vector<pair<int, int>>& point, int fit_order, Eigen::MatrixXd& W)
  {
    if (point.size() > fit_order)
    {
      Eigen::MatrixXd A(point.size(), fit_order + 1);
      Eigen::MatrixXd B(point.size(), 1);
      for (int i = 0;i < point.size();i++)
      {
        for (int j = 0;j <= fit_order;j++)
        {
          A(i, j) = pow(point[i].first, j);
        }
        B(i) = point[i].second;
      }
      W = (A.transpose() * A).inverse() * A.transpose() * B;
    }
    else
    {
      W = Eigen::MatrixXd::Zero(fit_order + 1, 1);
      ROS_WARN("fit_poly : too few points( %d ) to fit poly ", (int)point.size());
    }
  }

  void find_fence(cv::Mat& src)
  {
    clock_t start = clock();
    pair<int, int> left_right_index;
    vector<pair<int, int>> no_zero_index;
    get_hist_nozero(src, left_right_index, no_zero_index);

    cv::cvtColor(src, arbe_project_bgr, cv::COLOR_GRAY2BGR);
    arbe_project_fit = arbe_project_bgr.clone();
    arbe_project_detect = arbe_project_bgr.clone();

    if (left_right_index.second - left_right_index.first > (float)FENCE_MIN_DISTANCE / SPACIAL_RES)
    {
      vector<pair<int, int>>  left_point;
      vector<pair<int, int>>  right_point;
      get_fence_position(src, left_right_index, no_zero_index, left_point, right_point);

      Eigen::MatrixXd left_w, right_w;
      fit_poly(left_point, 2, left_w);
      fit_poly(right_point, 2, right_w);

      draw_fence_result(src, left_point, right_point, left_w, right_w);

      ROS_INFO("find_fence : left : (%f, %f, %f), right :  (%f, %f, %f)",
        left_w(0), left_w(1), left_w(2), right_w(0), right_w(1), right_w(2));
    }
    else
    {
      ROS_WARN("find_fence : can not find fence ");
    }

    cv::Mat tmp;
    cv::hconcat(arbe_project_bgr, arbe_project_detect, tmp);
    cv::hconcat(tmp, arbe_project_fit, arbe_project_output);
    // cv::imwrite("/home/qinguoyu/radar_slam/fence_result.jpg", arbe_project_fit);

    // cv::imshow("fence result", arbe_project_fit);
    // cv::waitKey();

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("find_fence : duration  %f", duration);
  }

  string int2str(int n)
  {
    ostringstream oss;
    oss << n;
    return oss.str();
  }


  void do_morphologyex(cv::Mat& src, cv::Mat& dst, int width)
  {
    clock_t start = clock();
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(width, width));
    cv::Mat kernel = (cv::Mat_<uchar>(3, 3) <<
      0, 1, 0,
      0, 1, 0,
      0, 1, 0);
    cv::morphologyEx(src, dst, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 10);

    // cv::Mat merge;
    // cv::hconcat(src, dst, merge);
    // cv::imwrite("/home/qinguoyu/radar_slam/morphologyex.jpg", merge);
    // cv::imshow("do_morphologyex", merge);
    // cv::waitKey();
    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("do_morphologyex : duration  %f", duration);
  }

  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {

    clock_t start = clock();

    copy_pointcloud(arbe_origin_ros);
    get_project_image();
    cv::threshold(arbe_project_gray, arbe_project_binary, 25, 255, cv::THRESH_BINARY);
    do_morphologyex(arbe_project_binary, arbe_project_close, 3);
    find_fence(arbe_project_close);
    publish();
    // reset_memery();

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("MutiPath : total duration  %f", duration);
  }

  void copy_pointcloud(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {
    clock_t start = clock();

    cloud_header = arbe_origin_ros->header;
    cloud_header.stamp = ros::Time::now();
    pcl::fromROSMsg(*arbe_origin_ros, *arbe_origin_pcl);

    Eigen::AngleAxisf rotation(0.1, Eigen::Vector3f::UnitX());
    Eigen::Translation3f translation(0, 0, 0);
    Eigen::Matrix4f rt = (translation * rotation).matrix();
    pcl::transformPointCloud(*arbe_origin_pcl, *arbe_origin_pcl, rt);

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("copy_pointcloud : get %d origin ROS point, duration %f ",
      arbe_origin_ros->width, duration);
  }

  void get_project_image()
  {
    clock_t start = clock();
    arbe_project_gray = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    unordered_map<pair<int, int>, int, PairHash> power_map;
    float max_power = 0;

    for (PointA point : arbe_origin_pcl->points)
    {
      if (abs(point.x) > X_MAX | abs(point.elevation - E_BIAS) > E_MAX)
      {
        cout << point.x << " " << point.elevation << " " << abs(point.elevation - E_BIAS) << "\\";
        continue;
      }
      int row = ROW - (int)(point.y / SPACIAL_RES);
      int col = (int)(point.x / SPACIAL_RES) + COL / 2;
      pair<int, int> row_col(row, col);
      if (power_map.find(row_col) == power_map.end())
      {
        power_map.emplace(row_col, (int)point.power_value);
      }
      else
      {
        power_map[row_col] += (int)point.power_value;
      }
      if (power_map[row_col] > max_power)
      {
        max_power = power_map[row_col];
      }
    }

    for (auto iter : power_map)
    {
      if (iter.second > (max_power / 10))
      {
        arbe_project_gray.at<uchar>(iter.first.first, iter.first.second) = iter.second * 255 / max_power;
      }
    }

    // cv::imwrite("/home/qinguoyu/radar_slam/project_gray.jpg", arbe_project_gray);
    // cout << "done project" << endl;
    // cv::imshow("project", arbe_project_gray);
    // cv::waitKey();

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("get_project_image : get %d project points, duration %f", (int)power_map.size(), duration);
  }

  void publish()
  {
    clock_t start = clock();

    arbe_project_image_msg.header.stamp = ros::Time::now();
    arbe_project_image_msg.header.seq += 1;
    cv::imencode(".jpg", arbe_project_output, arbe_project_image_msg.data);
    arbe_project_image_pub.publish(arbe_project_image_msg);

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("publish :  duration %f", duration);
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


  void reset_memery()
  {
    arbe_project_value = cv::Mat(ROW, COL * 2, CV_16UC1, cv::Scalar(0));
    arbe_project_image = cv::Mat(ROW, COL * 2, CV_8UC3, cv::Vec3b(0, 0, 0));
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mutipath");

  MutiPath mp;

  ros::spin();
}
