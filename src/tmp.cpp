#include <iostream>
#include <cstdlib>
#include <deque>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <string>
#include "../include/loc_ang.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include "xunji/ctrl_cmd.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

using namespace std;
#define PI 3.1415926

// ros中GPS相应数据
sensor_msgs::NavSatFix current_pos;

// 存放每一个小目标的经纬度
deque <pair<double,double>> aim_dis;
deque <pair<double,double>> pre_location;
double la_9,lo_9,la_6,lo_6,la_3,lo_3,la_0,lo_0;
double angle_1,angle_2,angle_3,tmp_angle;

// 这个只能先这样写了
void load_aim_dis() {
    aim_dis.push_back(make_pair(39.059070284, 117.130327259));
    aim_dis.push_back(make_pair(0, 0));
}

// 记录经纬度的值
double la = 0.0;
double lo = 0.0;
// 判断是否要前往下一个地点
bool new_go = true;
double pre_angle = 0; // 这个pre_angle是为了之后算的时候有可能出现的angle=nan准备的
// 设置一个量表示转弯，没转好之前不可以停下, turn_size表示要执行转弯的次数
bool start_turn = false;
int turn_size = 0, tmp_turn = 0, tmp_turn_size = 0,tmp_count = 8;
bool after_turn = false;

// 回调函数，获取当前的经纬度
void get_Now_Pos(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_pos = *msg;
    la = current_pos.latitude;  // 如39.058816632  放在前面
    lo = current_pos.longitude;  // 如117.130357386  放在后面
    return;
}


// 单线雷达的回调函数，这个东西是逆时针论的
vector<float> ranges;
double increase_angle = 0.0138396155089,angle_index = 0;
// 80度角所对应的弧度值对应的索引，280度角对应的弧度值对应的索引
int first_start_angle_index = 101,second_start_angle_index = 353,angle_count;
double pos_x,pos_y;
float scan_points[150][2] = {0};
float tmp_barrier_angle;
double barrier_middle_count = 0,barrier_left_count = 0,barrier_right_count = 0;

// 判断各个方向是否有障碍物，目前已经得到了障碍物的坐标，可以考虑用区间中障碍点的个数去判断
int barrier_index;
void Judge_barrier()
{
    barrier_right_count = 0,barrier_middle_count = 0,barrier_left_count = 0;
    for(barrier_index=0;barrier_index<angle_count;barrier_index++)
    {
        pos_x = scan_points[barrier_index][0],pos_y = scan_points[barrier_index][1];
        if(pos_x>0)
        {
            if(pos_x<0.45)
            {
                if(pos_y<3) // 中间的障碍物
                    barrier_middle_count++;
            }
            else
            {
                tmp_barrier_angle = atan((pos_x-0.45)/pos_y)*180.0/PI;
                if(tmp_barrier_angle<45)
                    if(((pos_x-0.45)*(pos_x-0.45)+pos_y*pos_y)<9)
                    {
                        if(tmp_barrier_angle<5)
                            barrier_right_count += 0.5;
                        else
                            barrier_right_count += 1;
                    }
            }
        }
        else
        {
            if(pos_x>-0.45)
            {
                if(pos_y<3) // 中间的障碍物
                    barrier_middle_count++;
            }
            else
            {
                pos_x = abs(pos_x);
                tmp_barrier_angle = atan((pos_x-0.45)/pos_y)*180.0/PI;
                if(tmp_barrier_angle<45)
                    if(((pos_x-0.45)*(pos_x-0.45)+pos_y*pos_y)<9)
                    {
                        if(tmp_barrier_angle<5)
                            barrier_left_count += 0.5;
                        else
                            barrier_left_count += 1;
                    }
            }
        }
    }
    cout << " left-middle-right: " << barrier_left_count << " " << barrier_middle_count << " " << barrier_right_count << " ";
}

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    ranges = msg->ranges;
    angle_index = 0,angle_count = 0;
    for(vector<float>::iterator it = ranges.begin();it!=ranges.end();it++)
    {
        if(!isnan(*it) && !isinf(*it) && angle_index<first_start_angle_index && ranges[angle_index]<5.0)
        {
            pos_x = 0 - ranges[angle_index]*sin(increase_angle*angle_index);
            pos_y = ranges[angle_index]*cos(increase_angle*angle_index);
            if(pos_x!=0 && pos_y!=0)
            {
                scan_points[angle_count][0] = pos_x;
                scan_points[angle_count][1] = pos_y;
                angle_count++;
            }
        }
        if(!isnan(*it) && !isinf(*it) && angle_index>second_start_angle_index && ranges[angle_index]<5.0)
        {
            pos_x = 0 - ranges[angle_index]*sin(increase_angle*angle_index);
            pos_y = ranges[angle_index]*cos(increase_angle*angle_index);
            if(pos_x!=0 && pos_y!=0)
            {
                scan_points[angle_count][0] = pos_x;
                scan_points[angle_count][1] = pos_y;
                angle_count++;
            }
        }
        angle_index++;
    }
    Judge_barrier();
}

// 有障碍物返回true，没有返回false
bool Judge_middle()
{
    return barrier_middle_count > 3;
}

bool Judge_left()
{
    return barrier_left_count > 3;
}

bool Judge_right()
{
    return barrier_right_count > 3;
}

/**
 * 返回现在应该转向的角度
 * 具体思路是算出之前三个位置的方向角度，然后求平均值，得到目前运动的方向角度，然后与传入的angle进行
 * 对比，如果需要向左转就返回一个正数，向右转就返回一个负数
 * */

int main(int argc, char **argv) {
    xunji::ctrl_cmd msg_run;
    msg_run.ctrl_cmd_gear = 2;
    msg_run.ctrl_cmd_velocity = 0.1;
    msg_run.ctrl_cmd_steering = 0;
    msg_run.ctrl_cmd_Brake = 0;

    xunji::ctrl_cmd msg_turn_left;
    msg_turn_left.ctrl_cmd_gear = 2;
    msg_turn_left.ctrl_cmd_velocity = 0.1;
    msg_turn_left.ctrl_cmd_steering = 30;
    msg_turn_left.ctrl_cmd_Brake = 0;

    xunji::ctrl_cmd msg_turn_right;
    msg_turn_left.ctrl_cmd_gear = 2;
    msg_turn_left.ctrl_cmd_velocity = 0.1;
    msg_turn_left.ctrl_cmd_steering = -30;
    msg_turn_left.ctrl_cmd_Brake = 0;


    xunji::ctrl_cmd msg_stop;
    msg_turn_left.ctrl_cmd_gear = 2;
    msg_turn_left.ctrl_cmd_velocity = 0;
    msg_turn_left.ctrl_cmd_steering = 0;
    msg_turn_left.ctrl_cmd_Brake = 0;

    // 下一个目标的经纬度
    double dis_la = aim_dis.front().first, dis_lo = aim_dis.front().second;
    // 下一个目标的角度，距离
    double angle, dis, aim_angle = 0;

    ros::init(argc, argv, "sub_node");

    ros::NodeHandle n("~");
    ros::Subscriber subScan = n.subscribe("/scan",1000,ScanCallback);

// 这里是不是应该重新订阅一个话题之类的，之后如果考虑做避障的话
// 避障的路径规划我考虑了一种想法，是不是检测到障碍物的时候判断左右，选择一个方向进行转弯，直到转至视野里没有障碍物为停止，此时再进行
// 角度计算，再转弯，但是这样要考虑马路牙子？而且还要考虑在转弯的时候遇见障碍物怎么办？

    // ros::Publisher pub_msg = n.advertise<xunji::ctrl_cmd>("/ctrl_cmd",100);

    int i = 0,tmp = 0,count = 90*0.6;
    // 每秒钟拿出4个当前位置点
    ros::Rate rate(3);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        cout << i << endl;
        i++;
        status = ros::ok();
        rate.sleep();
    }

    ros::spin();

    return 0;
}














