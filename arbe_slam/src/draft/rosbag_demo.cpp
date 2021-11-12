#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <thread>

ros::Publisher pub;
pcl::visualization::CloudViewer viewer("Cloud Viewer");

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  rosbag::Bag bag;
  bag.open("../2020-03-26-17-53-18/2020-03-26-17-53-18.bag", rosbag::bagmode::Read); //打开一个bag文件

  std::vector<std::string> topics; //设置需要遍历的topic
  topics.push_back(std::string("/left/rslidar_packets"));
  topics.push_back(std::string("/right/rslidar_packets"));
  topics.push_back(std::string("/usb_cam/image_raw0/compressed"));
  topics.push_back(std::string("/usb_cam/image_raw1/compressed"));
  topics.push_back(std::string("/usb_cam/image_raw2/compressed"));
  topics.push_back(std::string("/usb_cam/image_raw3/compressed"));
  topics.push_back(std::string("/miliwave_base"));
  topics.push_back(std::string("/back_miliwave_base"));
  //topics.push_back(std::string("/radar_cloud "));
  rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下
  //rosbag::View view_all(view); //读取全部topic


  // foreach(rosbag::MessageInstance const m, view) //用foreach遍历所有帧数据，每个messageInstance就是一帧数据
  // {
  int num;
  while (true) {
    rosbag::View::iterator it = view.begin(); //使用迭代器的方式遍历
    std::cout << "please input num:";
    std::cin >> num;
    std::cout << "num:" << num << std::endl;
    // for(int i = 0; i < num; ++i)
    //     ++it;
    std::advance(it, num);
    for (; it != view.end(); ++it)
    {
      auto m = *it;
      std::string topic = m.getTopic();
      std::cout << "topic:" << topic << std::endl;
      std::string callerid = m.getCallerId();
      std::cout << "callerid:" << callerid << std::endl;
      ros::Time  time = m.getTime();
      std::cout << "time:" << time.sec << ":" << time.nsec << std::endl;
      //ros::Time translated = time_translator_.translate(time);


      sensor_msgs::PointCloud2::ConstPtr input = m.instantiate<sensor_msgs::PointCloud2>();
      //sensor_msgs::PointCloud2ConstPtr input = *s ;
      if (input != NULL)
      {
        // 创建一个输出的数据格式
        sensor_msgs::PointCloud2 input;  //ROS中点云的数据格式
        //对数据进行处理
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(input, *cloud);

        //blocks until the cloud is actually rendered
        viewer.showCloud(cloud);

        //pub.publish (input);
      }
      if (topic == "/usb_cam/image_raw0/compressed") {
        sensor_msgs::CompressedImage::ConstPtr frontCamMsg = m.instantiate<sensor_msgs::CompressedImage>();
        //char* img = new char[frontCamMsg->data.size()];
        char img[frontCamMsg->data.size()];
        memcpy(img, &frontCamMsg->data[0], frontCamMsg->data.size());
        for (int i = 0; i < frontCamMsg->data.size(); ++i)
          std::cout << (int)img[i] << " ";
        std::cout << std::endl;
      }
      //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }
  }
  getchar();
  bag.close();
}