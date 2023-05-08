#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <std_msgs/Float32.h>

// float angle_increment = 0.0138396155089;
ros::Publisher distance_pub;
float calculateDistance(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  // Get scan parameters
  std::vector<float> ranges = scan_msg->ranges;
  float angle_increment = scan_msg->angle_increment;

  // Define parameters for detecting curb
  float curb_angle = 0.5;  // Angle of laser beam when pointed at curb
  float curb_tolerance = 0.1;  // Tolerance for detecting curb
  float max_range = 10.0;  // Maximum range to detect curb

  // Calculate number of points
  int num_points = ranges.size();

  // Initialize variables for curb detection
  bool on_curb = false;
  float curb_distance = 0.0;
  // std::cout<<"size:"<<num_points<<std::endl;
  // Define the start and end indices for the points we want to keep
  int start_index = static_cast<int>(num_points * 0.12);  // 0.12 represents 55/454
  int end_index = static_cast<int>(num_points * 0.375);  // 0.375 represents 170/454
  // Limit the range of points to keep
  start_index = std::max(0, start_index);
  end_index = std::min(num_points, end_index);

  // Cluster the points within the given range based on distance
  std::vector<std::vector<float>> clusters;
  float cluster_threshold = 0.05; // Maximum distance between points in a cluster
  std::vector<float> current_cluster;

  // Calculate coordinates of each point
  for (int i = start_index ; i < end_index; i++)
  {
    float range = ranges[i];
    if(isnan(range) || isinf(range) ) continue;

    // Calculate angle of current point
    float angle =  i * angle_increment;
    // ROS_INFO("i = %d Curb distance: %f ,angle: %f", i ,range,angle);

    // Calculate x and y coordinates of current point
    float x = angle;
    float y = range;
    float pos_x = ranges[i]*sin(angle_increment*i);
    float pos_y = ranges[i]*cos(angle_increment*i);

    // If the current cluster is empty, add the current point to it
    if (current_cluster.empty()) {
        current_cluster.push_back(range);
    }else{
      float distance_to_cluster = fabs(range - current_cluster.back());
      if (distance_to_cluster < cluster_threshold) {
          current_cluster.push_back(range);
      }
      else {
          clusters.push_back(current_cluster);
          current_cluster.clear();
          current_cluster.push_back(range);
      }
    }

    // Calculate the height of the laser beam above the ground
    // float height = max_range * std::tan(curb_angle - angle);
    // ROS_INFO("i = %d Curb dis tance: %f ,angle: %f,pos_x:%f,pos_y: %f", i ,range,angle , pos_x,pos_y);

    // // Check if point is on curb
    // if (std::abs(height - curb_distance) < curb_tolerance && range < max_range)
    // {
    //   // ROS_INFO("i = %d Curb distance: %f ", i ,range);
    //   on_curb = true;
    // }

    // // Check if point is off curb
    // if (std::abs(height - curb_distance) > curb_tolerance && on_curb)
    // {
    //   // Do something with the curb distance here
    //   // For example, print it to the console
    //   // ROS_INFO("i = %d Other distance: %f", i ,range);
    //   on_curb = false;
    // }

    // // Update curb distance if point is on curb
    // if (on_curb && range < curb_distance)
    // {
    //   curb_distance = range;
    // }

  }
  if (!current_cluster.empty()) {
        clusters.push_back(current_cluster);
    }
  // Find the largest cluster and calculate its median distance
  float largest_cluster_size = 0;
  std::vector<float> largest_cluster;
  for (const auto& cluster : clusters) {
      if (cluster.size() > largest_cluster_size) {
          largest_cluster_size = cluster.size();
          largest_cluster = cluster;
      }
  }

  // Calculate the median distance of the largest cluster
  float median_distance;
  std::size_t size = largest_cluster.size();
  if (size % 2 == 0) {
      median_distance = (largest_cluster[size / 2 - 1] + largest_cluster[size / 2]) / 2.0;
  }
  else {
      median_distance = largest_cluster[size / 2];
  }
  ROS_INFO("Curb dis tance: %f ,size: %d,num_points:%d" ,median_distance ,size,num_points);
  return median_distance;

}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  float distance = calculateDistance(scan_msg);
  std_msgs::Float32 distance_msg;

  distance_msg.data = distance;
  distance_pub.publish(distance_msg);

}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "roadside_dis");
  ros::NodeHandle nh;
  

  // Subscribe to scan topic
  ros::Subscriber sub_scan = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);
  distance_pub = nh.advertise<std_msgs::Float32>("distance_topic", 10);


  // Spin
  ros::spin();

  return 0;
}
