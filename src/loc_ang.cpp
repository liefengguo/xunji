#include "../include/loc_ang.h"
#include "math.h"
#include <iomanip>

using namespace std;

// PI
#define M_PI  3.14159265358979323846

// 地球半径
const double EARTH_RADIUS = 6371000;

// 大地坐标系资料WGS-84,长半径 6378137
const double WGS84_L_RADIUS = 6378137;

// 大地坐标系资料WGS-84,短半径 6356752.3142 */
const double WGS84_S_RADIUS = 6356752.3142;

// 扁率
const double M_F = 1 / 298.2572236;

// 角度转弧度
double A2R(double d) {
    return d * M_PI / 180.0; }

// 弧度转角度
double R2A(double d) {
    return d / M_PI * 180.0; }

// 1、计算两个经纬度之间的距离(m)
double GetDistanceCpp(double lat1, double lng1, double lat2, double lng2)
{

    double radLat1 = A2R(lat1);
    double radLat2 = A2R(lat2);
    double a = radLat1 - radLat2;
    double b = A2R(lng1) - A2R(lng2);
    double s = 2 * asin(sqrt(pow(sin(a/2),2) +cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS;
    return s;
}

// 2、计算两个经纬度之间的方位角(偏北角)(°)
double GetYaw(double lat1, double lon1, double lat2, double lon2)
{
    double result = 0.0;

    // 哎哎哎，这里我把int改成long，然后后面乘360就行了！！！太6了，哈哈哈哈
    long ilat1 = (long ) ((0.50 + lat1 * 360000.0)*360);
    long ilat2 = (long ) ((0.50 + lat2 * 360000.0)*360);
    long ilon1 = (long ) ((0.50 + lon1 * 360000.0)*360);
    long ilon2 = (long ) ((0.50 + lon2 * 360000.0)*360);

    // 这里也是，漂亮，原来的nan现在也有数值了
    lat1 = A2R(lat1)*360;
    lon1 = A2R(lon1)*360;
    lat2 = A2R(lat2)*360;
    lon2 = A2R(lon2)*360;

    if ((ilat1 == ilat2) && (ilon1 == ilon2)){
        return result;}
    else if (ilon1 == ilon2){
        if (ilat1 > ilat2){
            result = 180.0;}}
    else
    {
       //  cout << setprecision(8) << lon1 << " " << lon2 << " " << lat1 << " " << lat2 << endl;
        double c = acos(sin(lat2) * sin(lat1) + cos(lat2)* cos(lat1) * cos((lon2 - lon1)));
        double A = asin(cos(lat2) * sin((lon2 - lon1)) / sin(c));

        result = R2A(A);

        if ((ilat2 > ilat1) && (ilon2 > ilon1)){
        }
        else if ((ilat2 < ilat1) && (ilon2 < ilon1)){
            result = 180.0 - result;}
        else if ((ilat2 < ilat1) && (ilon2 > ilon1)){
            result = 180.0 - result;}
        else if ((ilat2 > ilat1) && (ilon2 < ilon1)){
            result += 360.0;}
    }
    if(result<0){
        result +=360.0;}
    if(result>360){
        result -=360.0;}

    return result;
}


//int main(int argc, char **argv)
//{
//    double pre_la = 39.059497847,pre_lo = 117.130564334,la = 39.059498076,lo = 117.13058769;
//    cout << GetYaw(pre_la,pre_lo,la,lo) << endl;
//    return 0;
//}










