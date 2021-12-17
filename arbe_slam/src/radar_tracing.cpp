#include "utility.h"

class RadarTracing
{
private:
  ros::NodeHandle nh;
  ros::Subscriber arbe_object_sub;
  ros::Publisher arbe_object_trace_pub;
  ros::Publisher arbe_object_predict_pub;
  ros::Publisher arbe_object_trace_valid_pub;

  PointCloudAO::Ptr arbe_object_input_pcl;
  PointCloudAO::Ptr arbe_object_trace_pcl;
  PointCloudAO::Ptr arbe_object_predict_pcl;
  PointCloudAO::Ptr arbe_object_trace_valid_pcl;

  bool is_init = false;

  double dt;
  double pre_time;

  unordered_map<int, ExtendKalmanFilter, IntHash> ekf_map;

  queue<int> object_label_queue;

  std_msgs::Header cloud_header;

  bool trace_predict_flag = false;

  Eigen::MatrixXd correlation_matrix;


  vector<int> predict_input_match;


public:
  RadarTracing() :nh("~")
  {
    arbe_object_sub = nh.subscribe<sensor_msgs::PointCloud2>(arbe_object_topic, 1,
      &RadarTracing::object_callback, this);
    arbe_object_trace_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_object_trace_topic, 1);
    arbe_object_predict_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_object_predict_topic, 1);
    arbe_object_trace_valid_pub = nh.advertise<sensor_msgs::PointCloud2>(arbe_object_trace_valid_topic, 1);

    dt = 0.7;

    for (int i = 0;i < 100;i++)
    {
      object_label_queue.push(i);
    }

    allocate_memory();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  void allocate_memory()
  {
    arbe_object_input_pcl.reset(new PointCloudAO);
    arbe_object_trace_pcl.reset(new PointCloudAO);
    arbe_object_predict_pcl.reset(new PointCloudAO);
    arbe_object_trace_valid_pcl.reset(new PointCloudAO);
  }


  void object_callback(const sensor_msgs::PointCloud2ConstPtr& arbe_object_input_ros)
  {
    clock_t start = clock();
    copy_pointcloud(arbe_object_input_ros);
    cout << "copy_pointcloud" << endl;

    match_trace_to_input();
    cout << "match_trace_to_input" << endl;

    put_no_match_input_into_trace();
    cout << "put_no_match_input_into_trace" << endl;

    publish_object_trace();
    cout << "publish_object_trace" << endl;

    clock_t end = clock();
    double duration = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("RadarTracing : total duration  %f", duration);

  }

  void copy_pointcloud(const sensor_msgs::PointCloud2ConstPtr& arbe_object_input_ros)
  {
    pcl::fromROSMsg(*arbe_object_input_ros, *arbe_object_input_pcl);
    cloud_header = arbe_object_input_ros->header;
  }


  void match_trace_to_input()
  {
    if (arbe_object_trace_pcl->size() == 0)
    {
      return;
    }

    predict_input_match = vector<int>(arbe_object_predict_pcl->size(), -1);
    if (arbe_object_input_pcl->size() != 0)
    {
      calculate_correlation_matrix();
      find_predict_input_match();
    }

    trace_manergement();
    delet_invalid_trace_object();
  }

  void trace_manergement()
  {
    for (int index_trace = 0; index_trace < arbe_object_trace_pcl->size();index_trace++)
    {
      int index_match = predict_input_match[index_trace];
      auto object_trace = arbe_object_trace_pcl->begin() + index_trace;

      if (index_match != -1)
      {
        auto object_match = arbe_object_input_pcl->begin() + index_match;
        ROS_DEBUG("trace_manergement : match trace %d with input %d", index_trace, index_match);
        object_match->counter = 1;

        if (object_trace_ekf(object_trace, object_match))
        {
          if (object_trace->counter < 4)
          {
            object_trace->counter += 1;
          }
        }
        else
        {
          object_trace->counter -= 1;
        }
      }
      else
      {
        ROS_DEBUG("trace_manergement : can not match trace %d// label %d, counter %d, size %d", index_trace,
          object_trace->label, object_trace->counter,
          (int)arbe_object_trace_pcl->size());
        object_trace->counter -= 1;

        VectorXd z = VectorXd(3);
        z << object_trace->range, object_trace->azimuth, object_trace->doppler;
        ekf_map[object_trace->label].init();
        ekf_map[object_trace->label].feed(z, cloud_header.stamp.toSec());
      }
    }
  }

  void delet_invalid_trace_object()
  {
    for (int index_trace = 0; index_trace < arbe_object_trace_pcl->size();index_trace++)
    {
      if (arbe_object_trace_pcl->points[index_trace].counter <= 0)
      {
        auto object_trace = arbe_object_trace_pcl->begin() + index_trace;
        object_label_queue.push(arbe_object_trace_pcl->points[index_trace].label);
        ROS_DEBUG("trace_manergement : recycle %d, size %d",
          object_trace->label, (int)object_label_queue.size());

        ekf_map.erase(object_trace->label);
        arbe_object_trace_pcl->erase(object_trace);
        index_trace--;
      }
    }
  }

  void find_predict_input_match()
  {
    while (true)
    {
      Eigen::Index index_predict, index_input;
      double min_correlation = correlation_matrix.minCoeff(&index_predict, &index_input);
      if (min_correlation < MIN_COORELATION)
      {

        predict_input_match[index_predict] = index_input;
        cout << "find_predict_input_match : match predict " << index_predict << " with input " <<
          index_input << " correlation " << correlation_matrix(index_predict, index_input) << endl;

        for (int iter_predict = 0; iter_predict < arbe_object_predict_pcl->size();iter_predict++)
        {
          correlation_matrix(iter_predict, index_input) = 9999;
        }
        for (int iter_input = 0; iter_input < arbe_object_input_pcl->size();iter_input++)
        {
          correlation_matrix(index_predict, iter_input) = 9999;
        }
      }
      else
      {
        break;
      }
    }
  }

  void calculate_correlation_matrix()
  {

    correlation_matrix = Eigen::MatrixXd::Ones(arbe_object_predict_pcl->size(), arbe_object_input_pcl->size()) * 9999;
    cout << "calculate_correlation_matrix : " << arbe_object_predict_pcl->size() << " " << arbe_object_input_pcl->size() << endl;

    for (int index_predict = 0; index_predict < arbe_object_predict_pcl->size();index_predict++)
    {
      for (int index_input = 0; index_input < arbe_object_input_pcl->size();index_input++)
      {
        correlation_matrix(index_predict, index_input) = calculate_correlation(index_predict, index_input);
        // cout << "calculate_correlation_matrix : set " << index_predict << " " << index_input << " as " << correlation_matrix(index_predict, index_input) << endl;
      }
    }
    // cout << "calculate_correlation_matrix : " << correlation_matrix << endl;
  }

  double calculate_correlation(int index_predict, int index_input)
  {
    auto object_predict = arbe_object_predict_pcl->begin() + index_predict;
    auto object_input = arbe_object_input_pcl->begin() + index_input;

    double correlation = sqrt((object_predict->x - object_input->x) * (object_predict->x - object_input->x)
      + (object_predict->y - object_input->y) * (object_predict->y - object_input->y));
    return correlation;
  }

  void put_no_match_input_into_trace()
  {
    for (int i = 0;i < arbe_object_input_pcl->size();i++)
    {
      if (arbe_object_input_pcl->points[i].counter == 0)
      {
        int trace_szie = arbe_object_trace_pcl->size();

        arbe_object_trace_pcl->push_back(arbe_object_input_pcl->points[i]);
        auto object_new = arbe_object_trace_pcl->begin() + trace_szie;

        object_new->label = object_label_queue.front();
        object_new->counter = 1;
        object_label_queue.pop();

        ExtendKalmanFilter ekf_tmp;
        ekf_map[object_new->label] = ekf_tmp;

        VectorXd z = VectorXd(3);
        z << object_new->range, object_new->azimuth, object_new->doppler;
        VectorXd x_ekf = ekf_map[object_new->label].feed(z, cloud_header.stamp.toSec());

        ROS_DEBUG("put_no_match_input_into_trace : put input %d into trace list label %d, counter %d",
          i, object_new->label, object_new->counter);
      }
    }
  }

  bool object_trace_ekf(std::vector<PointAO, Eigen::aligned_allocator<PointAO>>::iterator object_trace
    , std::vector<PointAO, Eigen::aligned_allocator<PointAO>>::iterator object_match)
  {
    VectorXd z = VectorXd(3);
    z << object_match->range, object_match->azimuth, object_match->doppler;
    cout << object_match->range << " " << object_match->azimuth << " " << object_match->doppler << endl;
    VectorXd x_ekf = ekf_map[object_trace->label].feed(z, cloud_header.stamp.toSec());

    double range_ekf = sqrt(x_ekf[0] * x_ekf[0] + x_ekf[1] * x_ekf[1]);
    double azimuth_ekf = atan2(x_ekf[0], x_ekf[1]);
    while (azimuth_ekf > M_PI) azimuth_ekf -= 2 * M_PI;
    while (azimuth_ekf < -M_PI) azimuth_ekf += 2 * M_PI;

    double doppler_ekf;
    if (range_ekf > 0.0001)
      doppler_ekf = (x_ekf[0] * x_ekf[2] + x_ekf[1] * x_ekf[3]) / range_ekf;
    else
      doppler_ekf = 0;

    double range_diff = abs(range_ekf - object_trace->range);
    double azimuth_diff = abs(azimuth_ekf - object_trace->azimuth);

    cout << "range_diff : " << range_diff << " " << (range_diff < 4) <<
      " azimuth_diff : " << azimuth_diff << " " << (azimuth_diff < (3.0 * 3.1415 / 180.0)) << endl;

    if (range_diff < 4 && azimuth_diff < (3.0 * 3.1415 / 180.0))
    {
      object_trace->x = x_ekf[0];
      object_trace->y = x_ekf[1];

      object_trace->z = object_match->z;
      object_trace->range = range_ekf;
      object_trace->azimuth = azimuth_ekf;
      // arbe_object_trace_pcl->points[index_trace].doppler = doppler_ekf;
      object_trace->doppler = object_match->doppler;
      object_trace->power = object_match->power;

      object_trace->width = object_match->width;
      object_trace->length = object_match->length;
      object_trace->high = object_match->high;
      object_trace->quantity = object_match->quantity;
      object_trace->noise_range = range_diff + 0.2;
      object_trace->noise_azimuth = azimuth_diff + 1 / 180 * 3.1415;
      return true;
    }
    else
    {
      z << object_trace->range, object_trace->azimuth, object_trace->doppler;
      ekf_map[object_trace->label].init();
      ekf_map[object_trace->label].feed(z, cloud_header.stamp.toSec());

      z << object_match->range, object_match->azimuth, object_match->doppler;
      ekf_map[object_trace->label].feed(z, cloud_header.stamp.toSec());

      return false;
    }
  }

  void get_object_trace_valid()
  {
    arbe_object_trace_valid_pcl->clear();
    cout << "get_object_trace_valid (label counter) : ";
    for (PointAO point : arbe_object_trace_pcl->points)
    {
      cout << point.label << " " << point.counter << "//";
      if (point.counter >= 3)
      {
        arbe_object_trace_valid_pcl->push_back(point);
      }
    }
    cout << endl;
  }

  void get_object_predict()
  {
    if (arbe_object_predict_pcl->size() > 0)
      arbe_object_predict_pcl->clear();
    for (int index_trace = 0;index_trace < arbe_object_trace_pcl->size();index_trace++)
    {
      auto object_trace = arbe_object_trace_pcl->begin() + index_trace;

      arbe_object_predict_pcl->push_back(*object_trace);

      cout << object_trace->label << " ";
      VectorXd x_pre = ekf_map[object_trace->label].get_predict();
      arbe_object_predict_pcl->points[index_trace].x = x_pre[0];
      arbe_object_predict_pcl->points[index_trace].y = x_pre[1];
    }
    cout << endl;
  }

  void publish_object_trace()
  {
    sensor_msgs::PointCloud2 arbe_object_trace_ros;
    pcl::toROSMsg(*arbe_object_trace_pcl, arbe_object_trace_ros);
    arbe_object_trace_ros.header = cloud_header;
    arbe_object_trace_pub.publish(arbe_object_trace_ros);

    get_object_predict();
    sensor_msgs::PointCloud2 arbe_object_predict_ros;
    pcl::toROSMsg(*arbe_object_predict_pcl, arbe_object_predict_ros);
    arbe_object_predict_ros.header = cloud_header;
    arbe_object_predict_pub.publish(arbe_object_predict_ros);

    get_object_trace_valid();
    sensor_msgs::PointCloud2 arbe_object_trace_valid_ros;
    pcl::toROSMsg(*arbe_object_trace_valid_pcl, arbe_object_trace_valid_ros);
    arbe_object_trace_valid_ros.header = cloud_header;
    arbe_object_trace_valid_pub.publish(arbe_object_trace_valid_ros);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "radar_tracing");

  RadarTracing rt;

  ros::spin();
}