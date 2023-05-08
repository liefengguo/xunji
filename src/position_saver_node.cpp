#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
double save_interval; // 保存间隔时间
double last_save_time; // 上一次保存时间
struct NavSatFixSaver
{
  std::ofstream file;

  void callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    // std::cout<<"ok"<<std::endl;
    // 获取当前时间
    double current_time = ros::Time::now().toSec();
    if (current_time - last_save_time > save_interval){
      if (file.is_open())
      {
        // file << msg->latitude  << "," << msg->longitude << std::endl;
        file << std::fixed << std::setprecision(11) << msg->latitude << "," << msg->longitude << std::endl;
        last_save_time = current_time;
      }
      else
      {
        ROS_ERROR_STREAM("Unable to open file.");
      }
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
  nh.param<double>("position_saver_node/save_interval", save_interval, 1.0);

  NavSatFixSaver saver;
  saver.file.open(file_path, std::ios::out | std::ios::app);

  if (!saver.file.is_open())
  {
    ROS_ERROR_STREAM("Unable to open file: " << file_path);
    return -1;
  }

  ros::Subscriber sub = nh.subscribe<sensor_msgs::NavSatFix>(topic_name, 1, &NavSatFixSaver::callback, &saver);

  last_save_time = ros::Time::now().toSec();


  ros::spin();

  saver.file.close();

  return 0;
}
