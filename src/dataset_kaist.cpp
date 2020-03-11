#include "dataset_kaist.h"

void DatasetKaist::init()
{
    //check path is right or not

     std::ifstream f((data_folder_path_+"/sensor_data/data_stamp.csv").c_str());
     if(!f.good()){
        std::cout << "Please check file path. Input path is wrong" << std::endl;
        return;
     }
     f.close();

     //Read CSV file and make map
     FILE *fp;
     int64_t stamp;
     //data stamp data load

     fp = fopen((data_folder_path_+"/sensor_data/data_stamp.csv").c_str(),"r");
     char data_name[50];
     data_stamp_.clear();
     while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
   //    data_stamp_[stamp] = data_name;
       data_stamp_.insert( std::multimap<int64_t, std::string>::value_type(stamp, data_name));
     }
     std::cout << "Stamp data are loaded" << std::endl;
     fclose(fp);

     initial_data_stamp_ = data_stamp_.begin()->first - 1;
     last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;

     //Read altimeter data
     fp = fopen((data_folder_path_+"/sensor_data/altimeter.csv").c_str(),"r");
     double altimeter_value;
     Altimeter altimeter_data;
     altimeter_data_.clear();
     while(fscanf(fp,"%ld,%lf\n",&stamp,&altimeter_value) == 2){
       altimeter_data.data = altimeter_value;
       altimeter_data_[stamp] = altimeter_data;
     }
     std::cout << "Altimeter data are loaded" << std::endl;
     fclose(fp);

     //Read encoder data
     fp = fopen((data_folder_path_+"/sensor_data/encoder.csv").c_str(),"r");
     int64_t left_count, right_count;
     Encoder encoder_data;
     encoder_data_.clear();
     int encoder_stop_count = 0;
     while(fscanf(fp,"%ld,%ld,%ld\n",&stamp,&left_count,&right_count) == 3){
           encoder_stop_count++;

           encoder_data.left_count = left_count;
           encoder_data.right_count = right_count;
           encoder_data_[stamp] = encoder_data;
     }

   //  cout << stop_period_.size() << endl;
     std::cout << "Encoder data are loaded" << std::endl;
     fclose(fp);

     //Read fog data
     fp = fopen((data_folder_path_+"/sensor_data/fog.csv").c_str(),"r");
     float d_roll, d_pitch, d_yaw;
     Gyro fog_data;
     fog_data_.clear();
     while(fscanf(fp,"%ld,%f,%f,%f\n",&stamp,&d_roll,&d_pitch,&d_yaw) == 4){
       fog_data.wx = d_roll;
       fog_data.wy = d_pitch;
       fog_data.wz = d_yaw;
       fog_data_[stamp] = fog_data;
     }
     std::cout << "Fog data are loaded" << std::endl;
     fclose(fp);

     //Read gps data
     fp = fopen((data_folder_path_+"/sensor_data/gps.csv").c_str(),"r");
     double latitude, longitude, altitude, altitude_orthometric;
     double cov[9];
     GpsLow gps_data;
     gps_data_.clear();
     while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8]) == 13){
       gps_data.latitude = latitude;
       gps_data.longitude = longitude;
       gps_data.altitude = altitude;
       for(int i = 0 ; i < 9 ; i ++) gps_data.position_covariance[i] = cov[i];
       gps_data_[stamp] = gps_data;

     }
     std::cout << "Gps data are loaded" << std::endl;
     fclose(fp);

     //Read gps data
     fp = fopen((data_folder_path_+"/sensor_data/vrs_gps.csv").c_str(),"r");
     double x_coordinate, y_coordinate, horizental_precision, lat_std, lon_std, altitude_std, heading_magnet, speed_knot, speed_km;
     int fix_state, number_of_sat, heading_valid;
     char GNVTG_mode;
     GpsHigh vrs_data;
     vrs_data_.clear();
   //  gps_odometry_data_.clear();
     while(1){
       int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf,%lf,%lf,%lf,%d,%lf,%lf,%lf,%c,%lf\n",&stamp,&latitude,&longitude,&x_coordinate,
                  &y_coordinate,&altitude,&fix_state,&number_of_sat,&horizental_precision,&lat_std,&lon_std,&altitude_std,
                  &heading_valid,&heading_magnet,&speed_knot,&speed_km,&GNVTG_mode,&altitude_orthometric);
       if(length == 18){
           vrs_data.latitude = latitude;
           vrs_data.altitude_orthometric = altitude_orthometric;
           vrs_data.longitude = longitude;
           vrs_data.x_coordinate = x_coordinate;
           vrs_data.y_coordinate = y_coordinate;
           vrs_data.altitude = altitude;
           vrs_data.fix_state = fix_state;
           if(fix_state == 1) vrs_data.fix_state_str = "normal";
           if(fix_state == 4) vrs_data.fix_state_str = "fix";
           if(fix_state == 5) vrs_data.fix_state_str = "float";
           vrs_data.number_of_sat = number_of_sat;
           vrs_data.horizental_precision = horizental_precision;
           vrs_data.lat_std = lat_std;
           vrs_data.lon_std = lon_std;
           vrs_data.altitude_std = altitude_std;
           vrs_data.heading_valid = heading_valid;
           vrs_data.heading_magnet = heading_magnet;
           vrs_data.speed_knot = speed_knot;
           vrs_data.speed_km = speed_km;
           vrs_data.GNVTG_mode = GNVTG_mode;
           vrs_data_[stamp] = vrs_data;

       }else if(length == 17){
           vrs_data.latitude = latitude;
           vrs_data.longitude = longitude;
           vrs_data.x_coordinate = x_coordinate;
           vrs_data.y_coordinate = y_coordinate;
           vrs_data.altitude = altitude;
           vrs_data.fix_state = fix_state;
           if(fix_state == 1) vrs_data.fix_state_str = "normal";
           if(fix_state == 4) vrs_data.fix_state_str = "fix";
           if(fix_state == 5) vrs_data.fix_state_str = "float";
           vrs_data.number_of_sat = number_of_sat;
           vrs_data.horizental_precision = horizental_precision;
           vrs_data.lat_std = lat_std;
           vrs_data.lon_std = lon_std;
           vrs_data.altitude_std = altitude_std;
           vrs_data.heading_valid = heading_valid;
           vrs_data.heading_magnet = heading_magnet;
           vrs_data.speed_knot = speed_knot;
           vrs_data.speed_km = speed_km;
           vrs_data.GNVTG_mode = GNVTG_mode;
           vrs_data_[stamp] = vrs_data;

       }else{
           break;

       }
     }
     std::cout << "Vrs gps data are loaded" << std::endl;
     fclose(fp);


     //Read IMU data
     fp = fopen((data_folder_path_+"/sensor_data/xsens_imu.csv").c_str(),"r");
     double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
     Imu imu_data;
     imu_data_.clear();

     while(1){
       int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
       if(length != 8 && length != 17) break;
       if(length == 8){
            std::cerr << "imu only contain angles, no acc and gyro......!!!!!!!!!" << std::endl;
       }else if(length == 17){
         imu_data.qx = q_x;
         imu_data.qy = q_y;
         imu_data.qz = q_z;
         imu_data.qw = q_w;

         imu_data.wx = g_x;
         imu_data.wy = g_y;
         imu_data.wz = g_z;
         imu_data.ax = a_x;
         imu_data.ay = a_y;
         imu_data.az = a_z;

         imu_data.mx = m_x;
         imu_data.my = m_y;
         imu_data.mz = m_z;



         imu_data_[stamp] = imu_data;

       }
     }
     std::cout << "IMU data are loaded" << std::endl;
     fclose(fp);


}

void DatasetKaist::start()
{
    for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
       auto stamp = iter->first;

          if(iter->second.compare("altimeter") == 0 && callback_altimeter_!= nullptr){
             callback_altimeter_(stamp, altimeter_data_[stamp]);
          }else if(iter->second.compare("encoder") == 0 && callback_encoder_!= nullptr){
            callback_encoder_(stamp, encoder_data_[stamp]);
          }else if(iter->second.compare("fog") == 0 && callback_gyro_!= nullptr){
            callback_gyro_(stamp, fog_data_[stamp]);
          }else if(iter->second.compare("gps") == 0 && callback_gpsLow_!= nullptr){
            callback_gpsLow_(stamp, gps_data_[stamp]);
          }else if(iter->second.compare("vrs") == 0 && callback_gpsHigh_ != nullptr){
            callback_gpsHigh_(stamp, vrs_data_[stamp]);
          }else if(iter->second.compare("imu") == 0 && callback_imu_!= nullptr){
            callback_imu_(stamp, imu_data_[stamp]);
          }

    }
}
