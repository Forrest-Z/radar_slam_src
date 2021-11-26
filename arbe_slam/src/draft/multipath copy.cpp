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
  cv::Mat arbe_project_close;
  cv::Mat arbe_project_open;
  cv::Mat arbe_project_filted;
  cv::Mat arbe_project_canny;
  cv::Mat arbe_project_hough;
  cv::Mat arbe_project_fit;

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
    string name = "1637211796_952459125.pcd";
    string path = "/home/qinguoyu/radar_slam/";

    string filename = path + name;

    if (pcl::io::loadPCDFile<PointA>(filename, *arbe_origin_pcl) != -1)
    {

      get_project_image();
      cv::threshold(arbe_project_gray, arbe_project_binary, 25, 255, cv::THRESH_BINARY);
      do_close(arbe_project_binary, arbe_project_close, 9);
      find_fence(arbe_project_close);

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
          hist[col] += 1;
          no_zero_index.push_back(pair<int, int>(row, col));
        }
      }
    }
    int* left = max_element(hist, hist + COL / 2);
    int* right = max_element(hist + COL / 2, hist + COL - 1);

    left_right_index = pair<int, int>(left - hist, right - hist);
    cout << left - hist << " : " << *left << " " << right - hist << " : " << *right << endl;
  }

  void get_fence_position(
    cv::Mat& src,
    pair<int, int>& left_right_index,
    vector<pair<int, int>>& no_zero_index,
    vector<pair<int, int>>& left_point,
    vector<pair<int, int>>& right_point)
  {
    int left_col = left_right_index.first;
    int right_col = left_right_index.second;
    int iter_row = ROW - FENCE_WINDOW_HIGH;

    for (int window = 0;window < FENCE_WINDOW_NUMBER;window++)
    {
      vector<pair<int, int>> left_index;
      vector < pair<int, int>> right_index;
      for (auto iter : no_zero_index)
      {
        if (abs(iter.first - iter_row) <= FENCE_WINDOW_HIGH
          && abs(iter.second - left_col) <= FENCE_WINDOW_WIDTH)
        {
          left_index.push_back(iter);
        }

        if (abs(iter.first - iter_row) <= FENCE_WINDOW_HIGH
          && abs(iter.second - right_col) <= FENCE_WINDOW_WIDTH)
        {
          right_index.push_back(iter);
        }
      }

      if (left_index.size() > MIN_PIXEL)
      {
        int sum = 0;
        for (auto iter : left_index)
        {
          sum += iter.second;
        }
        left_col = sum / left_index.size();
        left_point.push_back(pair<int, int>(iter_row, left_col));
      }

      if (right_index.size() > MIN_PIXEL)
      {
        int sum = 0;
        for (auto iter : right_index)
        {
          sum += iter.second;
        }
        right_col = sum / right_index.size();
        right_point.push_back(pair<int, int>(iter_row, right_col));
      }

      iter_row -= 2 * FENCE_WINDOW_HIGH;
    }
  }

  void draw_fence_result(cv::Mat& src, vector<pair<int, int>>& left_point, vector<pair<int, int>>& right_point,
    Eigen::MatrixXd left_w, Eigen::MatrixXd right_w)
  {
    cv::Mat src_bgr;
    cv::cvtColor(src, src_bgr, cv::COLOR_GRAY2BGR);
    cv::Mat fence_result = src_bgr.clone();
    cv::Mat fence_fit = src_bgr.clone();
    for (auto iter : left_point)
    {
      cv::Rect rect(iter.second - FENCE_WINDOW_WIDTH, iter.first - FENCE_WINDOW_HIGH,
        FENCE_WINDOW_WIDTH * 2, FENCE_WINDOW_HIGH * 2);
      cv::rectangle(fence_result, rect, cv::Scalar(0, 0, 255));
      cv::circle(fence_result, cv::Point(iter.second, iter.first), 5, cv::Scalar(0, 0, 255), -1);
    }

    for (auto iter : right_point)
    {
      cv::Rect rect(iter.second - FENCE_WINDOW_WIDTH, iter.first - FENCE_WINDOW_HIGH,
        FENCE_WINDOW_WIDTH * 2, FENCE_WINDOW_HIGH * 2);
      cv::rectangle(fence_result, rect, cv::Scalar(255, 0, 0));
      cv::circle(fence_result, cv::Point(iter.second, iter.first), 5, cv::Scalar(255, 0, 0), -1);
    }

    for (int row = left_point[left_point.size() - 1].first;row < left_point[0].first;row++)
    {
      int col = int(left_w(2) * (float)pow(row, 2)
        + left_w(1) * (float)pow(row, 1)
        + left_w(0) * (float)pow(row, 0));
      fence_fit.at<cv::Vec3b>(row, col - 1) = cv::Vec3b(0, 0, 255);
      fence_fit.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 255);
      fence_fit.at<cv::Vec3b>(row, col + 1) = cv::Vec3b(0, 0, 255);
    }

    for (int row = right_point[right_point.size() - 1].first;row < right_point[0].first;row++)
    {
      int col = int(right_w(2) * (float)pow(row, 2)
        + right_w(1) * (float)pow(row, 1)
        + right_w(0) * (float)pow(row, 0));
      fence_fit.at<cv::Vec3b>(row, col - 1) = cv::Vec3b(255, 0, 0);
      fence_fit.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 0, 0);
      fence_fit.at<cv::Vec3b>(row, col + 1) = cv::Vec3b(255, 0, 0);
      cout << row << " " << col << "\\";
    }

    cv::waitKey();

    cv::Mat merge1;
    cv::hconcat(src_bgr, fence_result, merge1);
    cv::hconcat(merge1, fence_fit, arbe_project_fit);
    // cv::imwrite("/home/qinguoyu/radar_slam/fence_result.jpg", arbe_project_fit);

    // cv::imshow("fence result", merge2);
    // cv::waitKey();
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
    cout << "W = " << W << endl;
  }

  void find_fence(cv::Mat& src)
  {
    pair<int, int> left_right_index;
    vector<pair<int, int>> no_zero_index;
    get_hist_nozero(src, left_right_index, no_zero_index);

    vector<pair<int, int>>  left_point;
    vector<pair<int, int>>  right_point;
    get_fence_position(src, left_right_index, no_zero_index, left_point, right_point);


    Eigen::MatrixXd left_w, right_w;
    fit_poly(left_point, 2, left_w);
    fit_poly(right_point, 2, right_w);

    draw_fence_result(src, left_point, right_point, left_w, right_w);
  }

  void do_median_blur(cv::Mat& src, cv::Mat& dst)
  {
    cv::medianBlur(src, dst, 3);

    cv::Mat merge;
    cv::hconcat(src, dst, merge);
    cv::imwrite("/home/qinguoyu/radar_slam/median_blur.jpg", merge);
    cout << "done median_blur" << endl;
  }

  string int2str(int n)
  {
    ostringstream oss;
    oss << n;
    return oss.str();
  }

  void do_close(cv::Mat& src, cv::Mat& dst, int width)
  {

    cv::Mat dilate = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    do_dilate(src, dilate, width);

    cv::Mat erode = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    do_erode(dilate, erode, width);

    dst = erode;

    cv::Mat merge;
    cv::hconcat(src, dst, merge);
    cv::imwrite("/home/qinguoyu/radar_slam/close.jpg", merge);
    cout << "done close" << endl;
  }

  void do_open(cv::Mat& src, cv::Mat& dst, int width)
  {
    cv::Mat erode = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    do_erode(src, erode, width);

    cv::Mat dilate = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    do_dilate(erode, dilate, width);

    dst = dilate;

    cv::Mat merge;
    cv::hconcat(src, dst, merge);
    cv::imwrite("/home/qinguoyu/radar_slam/open.jpg", merge);
    cout << "done open" << endl;
  }

  void do_dilate(cv::Mat& src, cv::Mat& dst, int& width)
  {

    for (int r = width; r < ROW - width;r++)
    {
      for (int c = width; c < COL - width;c++)
      {
        if ((int)src.at<uchar>(r, c) == 255)
        {
          for (int dr = -1 * width;dr <= width;dr++)
          {
            for (int dc = -1 * width;dc <= width;dc++)
            {
              dst.at<uchar>(r + dr, c + dc) = 255;
            }
          }
        }
      }
    }
  }

  void do_erode(cv::Mat& src, cv::Mat& dst, int& width)
  {
    for (int r = width; r < ROW - width;r++)
    {
      for (int c = width; c < COL - width;c++)
      {
        if ((int)src.at<uchar>(r, c) == 255)
        {
          bool erode = true;;
          for (int dr = -1 * width;dr <= width;dr++)
          {
            for (int dc = -1 * width;dc <= width;dc++)
            {
              if ((int)src.at<uchar>(r + dr, c + dc) == 0)
              {
                erode = false;
                break;
              }
            }
            if (!erode)
            {
              break;
            }
          }
          if (erode)
          {
            dst.at<uchar>(r, c) = 255;
          }
          else
          {
            dst.at<uchar>(r, c) = 0;
          }
        }
      }
    }
  }


  void do_morphologyex(int width, cv::Mat output)
  {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(width, width));
    // cv::Mat element(width, width, cv::CV_8UC, cv::Scalar(1));
    cv::Mat arbe_project_closed;
    // cv::erode(arbe_project_gray, arbe_project_closed, kernel);
    // cv::dilate(arbe_project_gray, arbe_project_closed, kernel);
    // cv::morphologyEx(arbe_project_gray, arbe_project_closed, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(arbe_project_gray, arbe_project_closed, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 10);

    for (int row = 0;row < ROW; row++)
    {
      for (int col = 0;col < COL;col++)
      {
        if (arbe_project_value.at<int>(row, col) != 0)
        {
          output.at<uchar>(row, col + COL) = arbe_project_closed.at<uchar>(row, col);
        }
      }
    }
  }


  void do_hough(cv::Mat& src, cv::Mat& dst, int factor)
  {
    cv::Mat src_copy;
    cv::cvtColor(src, src_copy, cv::COLOR_GRAY2BGR);
    dst = src_copy.clone();

    vector<cv::Vec4i> lines;
    cv::HoughLinesP(src, lines,
      factor, 2 * 3.1415 / 180, 50,
      20, 6);

    cout << lines.size() << endl;

    for (size_t i = 0; i < lines.size(); i++)
    {
      cv::Vec4i l = lines[i];
      cout << l << "\\";
      cv::line(dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1);
    }
    cout << endl;

    cv::Mat merge;
    cv::hconcat(src_copy, dst, merge);
    cv::imwrite("/home/qinguoyu/radar_slam/hough.jpg", merge);
    cout << "done _median_blur" << endl;
  }


  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& arbe_origin_ros)
  {

    clock_t start = clock();

    copy_pointcloud(arbe_origin_ros);
    cout << "copy_pointcloud" << endl;

    get_project_image();
    cv::threshold(arbe_project_gray, arbe_project_binary, 25, 255, cv::THRESH_BINARY);
    do_close(arbe_project_binary, arbe_project_close, 9);
    find_fence(arbe_project_close);


    publish();
    cout << "publish" << endl;

    reset_memery();
    cout << "reset_memery" << endl;

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

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("copy_pointcloud : get %d origin ROS point, duration %f ",
      arbe_origin_ros->width, duration);
  }

  void get_project_image()
  {
    arbe_project_image = cv::Mat(ROW, COL, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat arbe_project_origin = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    arbe_project_gray = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));

    float max_power = 0;
    unordered_map<pair<int, int>, int, PairHash> power_map;
    for (PointA point : arbe_origin_pcl->points)
    {
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
        // arbe_project_gray.at<uchar>(iter.first.first, iter.first.second) = 255;
      }
    }

    cv::imwrite("/home/qinguoyu/radar_slam/project_gray.jpg", arbe_project_gray);
    cout << "done project" << endl;
    // cv::imshow("project", arbe_project_gray);
    // cv::waitKey();
  }

  void do_canny(cv::Mat& src, cv::Mat& dst)
  {
    cv::Canny(src, dst, 100, 200);

    cv::Mat merge;
    cv::hconcat(src, dst, merge);
    cv::imwrite("/home/qinguoyu/radar_slam/canny.jpg", merge);
    cout << "done canny" << endl;
  }

  void publish()
  {
    arbe_project_image_msg.header.stamp = ros::Time::now();
    arbe_project_image_msg.header.seq += 1;
    cv::imencode(".jpg", arbe_project_fit, arbe_project_image_msg.data);
    arbe_project_image_pub.publish(arbe_project_image_msg);
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


  // void draw_image(int row, int col, int value, cv::Mat& arbe_project_image)
  // {
  //   if (row < ROW && col < COL * 3)
  //   {
  //     int high = value * ROW / 255;
  //     for (int i = 0; i < high;i++)
  //     {
  //       arbe_project_image.at<cv::Vec3b>(ROW - i - 1, col) = color_list[value];
  //     }

  //     // arbe_project_image.at<cv::Vec3b>(row, col) = cv::Vec3b(value, value, value);
  //   }
  // }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mutipath");

  MutiPath mp;

  ros::spin();
}
