#include <iostream>
#include <deque>
#include <unistd.h>
#include "math.h"
#include <cmath>
#include "../include/loc_ang.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include "sensor_msgs/LaserScan.h"

using namespace std;
#define PI 3.1415926

// ros中GPS相应数据
sensor_msgs::NavSatFix current_pos;


// 存放每一个小目标的经纬度
deque <pair<double,double>> aim_dis;
deque <pair<double,double>> pre_location;
double la_9,lo_9,la_6,lo_6,la_3,lo_3,la_0,lo_0;
double angle_1,angle_2,angle_3,tmp_angle,true_angle=0.0;
// 这个true_angle是在下面测试时临时加的值，后来发现很有用，当队列里的转弯值过大的时候多半可能是错误的
// 因为我一直都是直线行走，所以这样加上去效果还可以

// 这个只能先这样写了
void load_aim_dis() {

    //aim_dis.push_back(make_pair(39.058712328, 117.130391891));
    //aim_dis.push_back(make_pair(39.058798806, 117.130331169));
// ------------------------------ 上面是从出发点之前的拐弯
    //aim_dis.push_back(make_pair(39.059032359, 117.130337657));
    //aim_dis.push_back(make_pair(39.059136819, 117.130365628));
    //aim_dis.push_back(make_pair(39.059306785, 117.130326288));
    
    
    aim_dis.push_back(make_pair(39.060180335,117.130027318));
    
    
    //aim_dis.push_back(make_pair(39.059480169, 117.130351141));
    //aim_dis.push_back(make_pair(39.059503959, 117.130434119));
    //aim_dis.push_back(make_pair(39.059539852, 117.130684711));
    //aim_dis.push_back(make_pair(39.059539495, 117.131107941));
    //aim_dis.push_back(make_pair(39.059505916, 117.131484378));

    aim_dis.push_back(make_pair(39.059537182, 117.132002841));
    aim_dis.push_back(make_pair(39.059397789, 117.132048301));
    //aim_dis.push_back(make_pair(39.059234097, 117.132047267));
    //aim_dis.push_back(make_pair(39.058996941, 117.132034358));
    aim_dis.push_back(make_pair(39.058744186, 117.132018874));
    aim_dis.push_back(make_pair(39.058707784, 117.131850092));
    aim_dis.push_back(make_pair(39.058654858, 117.131567478));
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
int turn_size = 0, tmp_turn = 0, tmp_turn_size = 0,tmp_count = 14;
bool after_turn = false,find_car = false;
// 通过取出经纬度差异来判断车的走向
double dx_la = 0,dx_lo = 0,dx_main = 0;

// RTK回调函数，获取当前的经纬度
void get_Now_Pos(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_pos = *msg;
    la = current_pos.latitude;  // 如39.058816632  放在前面
    lo = current_pos.longitude;  // 如117.130357386  放在后面
    return;
}

/**
 * 返回现在应该转向的角度
 * 具体思路是算出之前三个位置的方向角度，然后求平均值，得到目前运动的方向角度，然后与传入的angle进行
 * 对比，如果需要向左转就返回一个正数，向右转就返回一个负数
 * */

int get_rotation(double angle)
{
    la_9 = pre_location.at(9).first,lo_9 = pre_location.at(9).second;
    la_6 = pre_location.at(6).first,lo_6 = pre_location.at(6).second;
    la_3 = pre_location.at(3).first,lo_3 = pre_location.at(3).second;
    la_0 = pre_location.at(0).first,lo_0 = pre_location.at(0).second;
    angle_1 = GetYaw(la_0,lo_0,la_9,lo_9);
    angle_2 = GetYaw(la_3,lo_3,la_9,lo_9);
    angle_3 = GetYaw(la_6,lo_6,la_9,lo_9);

    if(isnan(angle_1) || isnan(angle_2) || isnan(angle_3))
        return 400;
   /// 判断是往哪个方向行驶，东西南北
    dx_la = (la_9-la_0)*1000,dx_lo = (lo_9-lo_0)*1000;
    if(abs(dx_la)> abs(dx_lo))
        dx_main = dx_la;
    else
        dx_main = dx_lo;
    // ------------------------------------------------------------------------
    if(dx_main==dx_la)
    {
        if(dx_la>0)  // 从南向北方行驶，貌似和从西向东行驶一样
        {
            if(angle_1>180)
                angle_1 = angle_1-360;
            if(angle_2>180)
                angle_2 = angle_2-360;
            if(angle_3>180)
                angle_3 = angle_3-360;
            tmp_angle = (angle_1*4+angle_2*5+angle_3*1)/10;
            if(tmp_angle>180)
                tmp_angle = tmp_angle-360;
            if(angle>180)
                angle = angle-360;
            if(abs(angle_1-angle_2)<30&&abs(angle_2-angle_3)<30)
                true_angle = tmp_angle;
            else
                tmp_angle = true_angle;
            cout << "从南到北角度计算中的：" << angle_1 << " " << angle_2 << " " << angle_3 << " " << tmp_angle << " " << angle << endl;
            return (int)(tmp_angle-angle);
        }
        else
        {
            tmp_angle = (angle_1*4+angle_2*5+angle_3)/10;
            if(abs(angle_1-angle_2)<35&& abs(angle_2-angle_3)<35)
                true_angle = tmp_angle;
            else
                tmp_angle = true_angle;
            cout << "从北到南角度计算中的：" << angle_1 << " " << angle_2 << " " << angle_3 << " " << tmp_angle << " " << angle << endl;
            return (int)(tmp_angle-angle);
        }
    }
    else
    {
        if(dx_lo>0)
        {
            tmp_angle = (angle_1*4+angle_2*5+angle_3)/10;
            if(abs(angle_1-angle_2)<30&& abs(angle_2-angle_3)<30)
                true_angle = tmp_angle;
            else
                tmp_angle = true_angle;
            cout << "从西到东角度计算中的：" << angle_1 << " " << angle_2 << " " << angle_3 << " " << tmp_angle << " " << angle << endl;
            return (int)(tmp_angle-angle);
        }
        else
        {
            tmp_angle = (angle_1*4+angle_2*5+angle_3)/10;
            if(abs(angle_1-angle_2)<30&& abs(angle_2-angle_3)<30)
                true_angle = tmp_angle;
            else
                tmp_angle = true_angle;
            cout << "从东到西角度计算中的：" << angle_1 << " " << angle_2 << " " << angle_3 << " " << tmp_angle << " " << angle << endl;
            return (int)(tmp_angle-angle);
        }
    }
    return 400;
}

/// --------------------------------------------------------

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
int barrier_index,car_detect= 0;


int barrier_time_count = 0;
bool deal_barrier = false,in_three_second = false,re_car_detect = false;
bool after_avoid_barrier = false;
int pre_tmp_turn_size = 0,pre_turn_size = 0;
int car_barrier_count = 0,car_barrier_direction = 0;
bool restart_car_detect = false,last_car_barrier_count = 0;

void Judge_barrier()
{
    car_detect = 0;
    barrier_right_count = 0,barrier_middle_count = 0,barrier_left_count = 0;
    for(barrier_index=0;barrier_index<angle_count;barrier_index++)
    {
        pos_x = scan_points[barrier_index][0],pos_y = scan_points[barrier_index][1];
        if(pos_x>0)
        {
            if(pos_x<0.4)
            {
                if(pos_y<3.0) // 中间的障碍物
                    barrier_middle_count++;
                if(pos_y>5.0)
                    car_detect++;
            }
            else
            {
                tmp_barrier_angle = atan((pos_x-0.4)/pos_y)*180.0/PI;
                if(tmp_barrier_angle<55)   // 这里是检测两边55度角，并且在3米以内的障碍物
                {
                    if(((pos_x-0.4)*(pos_x-0.4)+pos_y*pos_y)<9.1)
                    {
                        if(tmp_barrier_angle<5)
                            barrier_right_count += 0.5;
                        else
                            barrier_right_count += 1;
                    }
                }
            }
        }
        else
        {
            if(pos_x>-0.4)
            {
                if(pos_y<3.0) // 中间的障碍物
                    barrier_middle_count++;
                if(pos_y>5.0)
                    car_detect++;
            }
            else
            {
                pos_x = abs(pos_x);
                tmp_barrier_angle = atan((pos_x-0.4)/pos_y)*180.0/PI;
                if(tmp_barrier_angle<55)
                {
                    if(((pos_x-0.4)*(pos_x-0.4)+pos_y*pos_y)<9.1)
                    {
                        if(tmp_barrier_angle<5)
                            barrier_left_count += 0.5;
                        else
                            barrier_left_count += 1;
                    }
                }
            }
        }
    }
}

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    ranges = msg->ranges;
    angle_index = 0,angle_count = 0;
    for(vector<float>::iterator it = ranges.begin();it!=ranges.end();it++)
    {
        if(!isnan(*it) && !isinf(*it) && angle_index<first_start_angle_index && ranges[angle_index]<7.0)
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
        if(!isnan(*it) && !isinf(*it) && angle_index>second_start_angle_index && ranges[angle_index]<7.0)
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

int car_left_count = 0,car_right_count = 0;
bool detect_car()
{
    car_left_count = 0,car_right_count = 0;
    for(barrier_index=0;barrier_index<angle_count;barrier_index++)
    {
        pos_x = scan_points[barrier_index][0],pos_y = scan_points[barrier_index][1];
        if(pos_x>0)
        {
            tmp_barrier_angle = atan((pos_x-0.4)/pos_y)*180.0/PI;
            if(tmp_barrier_angle>=30 && tmp_barrier_angle<=60)
                car_right_count++;
        }
        else
        {
            pos_x = abs(pos_x);
            tmp_barrier_angle = atan((pos_x-0.4)/pos_y)*180.0/PI;
            if(tmp_barrier_angle>=30 && tmp_barrier_angle<=60)
                car_left_count++;
        }
        // cout << car_left_count << " " << car_right_count << "  ";
    }
    if(car_barrier_direction>0)
    {
        if(car_right_count>=20)
            return false;
    }
    else
    {
        if(car_left_count>=20)
            return false;
    }
    return true; //  true就表示没有车了
}



int main(int argc, char **argv) {
    load_aim_dis();
    
    // 下一个目标的角度，距离
    double angle, dis, dis_la, dis_lo,aim_angle = 0;
    long i = 0;

    ros::init(argc, argv, "sub_node");

    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe("/fixposition/navsatfix", 1000, get_Now_Pos);
    ros::Subscriber subScan = n.subscribe("/scan",1000,ScanCallback);

    // 这里是不是应该重新订阅一个话题之类的，之后如果考虑做避障的话
    // 避障的路径规划我考虑了一种想法，是不是检测到障碍物的时候判断左右，选择一个方向进行转弯，直到转至视野里没有障碍物为停止，此时再进行
    // 角度计算，再转弯，但是这样要考虑马路牙子？而且还要考虑在转弯的时候遇见障碍物怎么办？

    // 这个只适合于静态障碍物，如果遇到动态障碍物怎么办？可以判断一下，如果10s以后障碍物位置还是不动，就认为它是静态的，在转弯，动态的就一直不动  
    // 用来发布话题的
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist msg_go_slower;
    msg_go_slower.linear.x = 0.1, msg_go_slower.linear.y = 0, msg_go_slower.linear.z = 0;
    msg_go_slower.angular.x = 0, msg_go_slower.angular.y = 0, msg_go_slower.angular.z = 0;

    geometry_msgs::Twist msg_go_slow;
    msg_go_slow.linear.x = 0.2, msg_go_slow.linear.y = 0, msg_go_slow.linear.z = 0;
    msg_go_slow.angular.x = 0, msg_go_slow.angular.y = 0, msg_go_slow.angular.z = 0;

    geometry_msgs::Twist msg_go;
    msg_go.linear.x = 0.3, msg_go.linear.y = 0, msg_go.linear.z = 0;
    msg_go.angular.x = 0, msg_go.angular.y = 0, msg_go.angular.z = 0;

    geometry_msgs::Twist msg_left_turn;
    msg_left_turn.linear.x = 0.2, msg_left_turn.linear.y = 0, msg_left_turn.linear.z = 0;
    msg_left_turn.angular.x = 0, msg_left_turn.angular.y = 0, msg_left_turn.angular.z = 0.1;

    geometry_msgs::Twist msg_right_turn;
    msg_right_turn.linear.x = 0.2, msg_right_turn.linear.y = 0, msg_right_turn.linear.z = 0;
    msg_right_turn.angular.x = 0, msg_right_turn.angular.y = 0, msg_right_turn.angular.z = -0.1;

    geometry_msgs::Twist msg_stop;
    msg_stop.linear.x = 0, msg_stop.linear.y = 0, msg_stop.linear.z = 0;
    msg_stop.angular.x = 0, msg_stop.angular.y = 0, msg_stop.angular.z = 0;

    // 每秒钟拿出3个当前位置点
    ros::Rate rate(3);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        // 打印出当前小车的经纬度
        cout.precision(12);
        cout << la << "||" << lo << "-------";
	// 下一个目标的经纬度
        dis_la = aim_dis.front().first, dis_lo = aim_dis.front().second;
        
        // 获取当前位置与目的地的距离与方向
        angle = GetYaw(la, lo, dis_la, dis_lo);
        dis = GetDistanceCpp(la, lo, dis_la, dis_lo);

        // 将之前的位置插入到deque中，deque长度最大保持10
        if(pre_location.size()>10)
            pre_location.pop_front();

        // 如果这个角度不是数字，就用上一个位置的角度来近似
        if (isnan(angle))
            angle = pre_angle;
        cout << angle << "  " << dis << endl;
        if( i<5 ) // 这里进行一个初始化的操作
        {
            pub.publish(msg_go_slower); // 让小车往前走
            pre_location.push_back(make_pair(la,lo));
        }
        else if (i>=5 && i<10)
        {
            pub.publish(msg_go_slow); // 让小车往前走
            pre_location.push_back(make_pair(la,lo));
        }
        else if (i>=10 && i < 22) // 这里进行一个初始化的操作
        {
            pub.publish(msg_go); // 让小车往前走
            pre_location.push_back(make_pair(la,lo));
        }
        else // 初始化结束
        {
            if (new_go) {

                if (dis_la == 0 || dis_lo == 0) // 到了最后一个，就让它停止
                {
                    pub.publish(msg_stop);
                    break;
                }
                new_go = false;
                // 准备计算角度，进行转弯，下一行是获取上一时刻的运动方向，然后与目的地做差
                start_turn = true;
                turn_size = get_rotation(angle);
                if (turn_size == 400 || abs(turn_size)>100)
                {
                    cout << "初始化失败，继续初始化！" << endl;
                    start_turn = false;
                    new_go = true;
                }
                else {
                    cout << "执行的是新的目标点里的：" << turn_size << endl;
                    if (turn_size > 0)
                        turn_size = turn_size * 0.58;
                    else
                        turn_size = turn_size * 0.51;
                    tmp_turn_size = abs(turn_size);
                    pre_turn_size = turn_size;
                    pre_tmp_turn_size = tmp_turn_size;
                }
            }
            else
            {
                // 如果贴边行走检测到车了，提前转弯吧
                if((car_detect>=5 && find_car==false) || re_car_detect == true)
                {
                    find_car = true,re_car_detect = false;
                    car_barrier_count = 0;
                    cout << "检测到车车了，提前转弯！并执行一系列特殊的操作" << endl;
                    if(!Judge_right() && !Judge_left())
                    {
                        cout << "两边安全，默认向右" << endl;
                        car_barrier_direction = -1;  // 实际上这个默认转向方向是要通过底盘上的雷达判断路边方向然后取一个相反方向
                    }
                    else if(!Judge_right() && Judge_left())
                    {
                        cout << "右面安全" << endl;
                        car_barrier_direction = -1;
                    }
                    else if(Judge_right() && !Judge_left())
                    {
                        cout << "左面安全" << endl;
                        car_barrier_direction = 1;
                    }
                    else
                    {
                        cout << "两边都不安全，原地待命" << endl;
                        car_barrier_direction = 0;
                    }
                }
                else if(find_car == true)
                {
                    
                    car_barrier_count++;
                    if(car_barrier_direction==0)
                    {
                        if(car_barrier_count==12)
                            re_car_detect = true;
                    }
                    else // 这里就是单纯的避车操作，不受别的影响
                    {
                        if(car_barrier_count<16) // 转向
                        {
                            cout << "执行这里" << " ";
                            last_car_barrier_count = 0;
                            if(car_barrier_count<0)
                                pub.publish(msg_left_turn);
                            else
                                pub.publish(msg_right_turn);
                        }
                        else if(car_barrier_count>=16 && car_barrier_count<44) // 直行
                            pub.publish(msg_go);
                        else if(car_barrier_count>=44 && car_barrier_count<62) // 回正
                        {
                            if(car_barrier_count<0)
                                pub.publish(msg_right_turn);
                            else
                                pub.publish(msg_left_turn);
                        }
                        else if(car_barrier_count>=62 && car_barrier_count<110) // 直行一段距离
                            pub.publish(msg_go);
                        else // 检测是否还有车辆，否则直行50
                        {
                            if(restart_car_detect== true && detect_car()== true) // 这就算摆脱车辆了
                            {
                                cout << "摆脱车辆了！！！" << endl;
                                find_car = false;
                            }
                            else if(restart_car_detect == true && detect_car()== false)
                                restart_car_detect = false;
                            else
                            {
                                if((car_barrier_count-last_car_barrier_count)>=55)
                                    restart_car_detect = true;
                                else
                                    pub.publish(msg_go);
                            }
                        }
                    }
                }
                else if((Judge_middle() && deal_barrier == false && find_car == false)|| (in_three_second== true && find_car == false))
                {
                    if(barrier_time_count==0)
                    {
                        cout << "检测到障碍物啦" << endl;
                        pub.publish(msg_stop);
                        in_three_second = true;
                    }
                    if(barrier_time_count==9)
                    {
                        deal_barrier = true;
                        in_three_second = false;
                        barrier_time_count = -1;
                        if(Judge_middle()) // 此时障碍物仍存在
                        {
                            if(!Judge_right() && !Judge_left())
                            {
                                cout << "两边安全，默认向左" << endl;
                                start_turn = true;
                                tmp_turn_size = 13;
                                turn_size = -13;
                                pre_turn_size = turn_size;
                                pre_tmp_turn_size = tmp_turn_size;
                            }
                            else if(!Judge_right() && Judge_left())
                            {
                                cout << "右面安全" << endl;
                                start_turn = true;
                                tmp_turn_size = 13;
                                turn_size = -13;
                                pre_turn_size = turn_size;
                                pre_tmp_turn_size = tmp_turn_size;
                            }
                            else if(Judge_right() && !Judge_left())
                            {
                                cout << "左面安全" << endl;
                                start_turn = true;
                                tmp_turn_size = 13;
                                turn_size = 13;
                                pre_turn_size = turn_size;
                                pre_tmp_turn_size = tmp_turn_size;
                            }
                            else
                            {
                                cout << "两边都不安全，原地待命" << endl;
                                deal_barrier = false;
                                in_three_second = true;
                            }
                        }
                        else // 障碍物不在，恢复原来的运动状态
                        {
                            start_turn = false;
                            tmp_count = 13;
                            tmp_turn_size = 0;
                            after_turn = true;
                        }
                    }
                    barrier_time_count++;
                }
                else // 正常情况下，应该保持前进，然后检查角度准备进行转弯
                {
                    if (start_turn) {
                        if (tmp_turn_size == 0) {
                            cout << "转完了一次" << endl;
                            start_turn = false;
                            after_turn = true;
                            tmp_count = 13;
                            continue;
                        }
                        if (turn_size > 0)
                            pub.publish(msg_left_turn);
                        else
                            pub.publish(msg_right_turn);
                        tmp_turn_size--;
                    }
                    else if (after_turn)
                    {
                        // 转弯后的直线行走完成，计算是否偏了
                        if(tmp_count==0)
                        {
                            after_turn = false;
                            tmp_count = 13;
                            pub.publish(msg_go);
                        }
                        pre_location.push_back(make_pair(la,lo));
                        pub.publish(msg_go);
                        tmp_count--;
                    }
                    else
                    {
                        deal_barrier = false;
                        // 正常行走时计算是否偏了
                        tmp_turn = get_rotation(angle);
                        // 这个就是避大弯的时候走偏了，就重新转回去
                        if(abs(tmp_turn)>120)
                        {
                            start_turn = true;
                            tmp_turn_size = pre_tmp_turn_size;
                            turn_size = 0 - pre_turn_size;
                            continue;
                        }
                        if (abs(tmp_turn) > 5 && tmp_turn != 400 && abs(tmp_turn)<150)
                        {
                            cout << "正常行走中判断角度里的：" << tmp_turn  << endl;
                            start_turn = true;
                            if (tmp_turn > 0)
                                turn_size = tmp_turn * 0.58;
                            else
                                turn_size = tmp_turn * 0.51;
                            tmp_turn_size = abs(turn_size);
                            pre_tmp_turn_size = tmp_turn_size;
                            pre_turn_size = turn_size;
                        } else
                        {
                            pre_location.push_back(make_pair(la,lo));
                            pub.publish(msg_go);
                        }
                    }
                }
            }
             // 如果距离目的地0.9m以内就进行目的地的更新，如果到了最后一个就停止  ----  这里暂定0.9m，之后视精度而定
            if (dis < 0.8) {
                new_go = true;
                pub.publish(msg_go_slower);
                aim_dis.pop_front();
            }
        }
        pre_angle = angle;
        i++;
        status = ros::ok();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
