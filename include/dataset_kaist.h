#ifndef DATASET_KAIST_H
#define DATASET_KAIST_H
#include<string>
#include <map>
#include <fstream>
#include <iostream>
#include <vector>
#include <functional>




#include "data_type.h"
class DatasetKaist
{
public:
    DatasetKaist(std::string data_dir):data_folder_path_(data_dir){
        init();
    };

    void init();
    void start();

    void regist_callback_altimeter(std::function<void(int64_t, Altimeter)> callback_altimeter){ callback_altimeter_ = callback_altimeter; }
    void regist_callback_encoder(std::function<void(int64_t, Encoder)> callback_encoder){ callback_encoder_ = callback_encoder; }
    void regist_callback_gyro(std::function<void(int64_t, Gyro)> callback_gyro){ callback_gyro_ = callback_gyro; }
    void regist_callback_gpsLow(std::function<void(int64_t, GpsLow)> callback_gpsLow){ callback_gpsLow_ = callback_gpsLow; }
    void regist_callback_gpsHigh(std::function<void(int64_t, GpsHigh)> callback_gpsHigh){ callback_gpsHigh_ = callback_gpsHigh; }
    void regist_callback_imu(std::function<void(int64_t, Imu)> callback_imu){ callback_imu_ = callback_imu; }
private:
    std::string data_folder_path_;

    std::multimap<int64_t, std::string>                    data_stamp_;
    std::map<int64_t, Altimeter>   altimeter_data_;
    std::map<int64_t, Encoder>     encoder_data_;
    std::map<int64_t, Gyro>   fog_data_;
    std::map<int64_t, GpsLow>    gps_data_;
    std::map<int64_t, GpsHigh>         vrs_data_;


    std::map<int64_t, Imu>         imu_data_;

    int64_t initial_data_stamp_;
    int64_t last_data_stamp_;

    bool auto_start_flag_;
    int stamp_show_count_;

    //-- callback
    std::function<void(int64_t, Altimeter)> callback_altimeter_;
    std::function<void(int64_t, Encoder)> callback_encoder_;
    std::function<void(int64_t, Gyro)> callback_gyro_;
    std::function<void(int64_t, GpsLow)> callback_gpsLow_;
    std::function<void(int64_t, GpsHigh)> callback_gpsHigh_;
    std::function<void(int64_t, Imu)> callback_imu_;

};

#endif // DATASET_KAIST_H
