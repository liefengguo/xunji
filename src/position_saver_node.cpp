#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>

struct NavSatFixSaver
{
  std::ofstream file;

  void callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    std::cout<<"ok"<<std::endl;
    if (file.is_open())
    {
      file << msg->latitude << "," << msg->longitude << std::endl;
    }
    else
    {
      ROS_ERROR_STREAM("Unable to open file.");
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_saver_node");

  ros::NodeHandle nh;

  std::string file_path, topic_name;
  nh.param<std::string>("position_saver_node/file_path", file_path, "/home/user/position.txt");
  nh.param<std::string>("position_saver_node/topic_name", topic_name, "/fixposition/navsatfix");

  NavSatFixSaver saver;
  saver.file.open(file_path, std::ios::out | std::ios::app);

  if (!saver.file.is_open())
  {
    ROS_ERROR_STREAM("Unable to open file: " << file_path);
    return -1;
  }

  ros::Subscriber sub = nh.subscribe<sensor_msgs::NavSatFix>(topic_name, 1, &NavSatFixSaver::callback, &saver);

  ros::spin();

  saver.file.close();

  return 0;
}
