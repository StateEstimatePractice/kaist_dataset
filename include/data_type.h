#ifndef DATA_TYPE_H
#define DATA_TYPE_H
#include <string>

struct Imu
{
    double wx,wy,wz; //-- 角速度
    double ax,ay,az; //-- 加速度
    double mx,my,mz; //-- 磁
    double qw,qx,qy,qz;
};

struct Gyro
{
    double wx;
    double wy;
    double wz;
};

struct GpsLow
{
    double latitude;
    double longitude;
    double altitude;

    double position_covariance[9];
};

struct GpsHigh
{
    double latitude;
    double longitude;
    double altitude;

    double x_coordinate, y_coordinate;

     double altitude_orthometric;

   double horizental_precision, lat_std, lon_std, altitude_std, heading_magnet, speed_knot, speed_km;
   int fix_state, number_of_sat, heading_valid;
    char GNVTG_mode;

    std::string fix_state_str;
};

struct Encoder
{
  long left_count;
  long right_count;
};

struct Altimeter
{
    double data;
};

#endif // DATA_TYPE_H
