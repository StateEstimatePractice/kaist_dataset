#include "dataset_kaist.h"


int main()
{

    DatasetKaist data_("/home/wk/Dataset/kaist/23/urban23-highway");

    data_.regist_callback_imu([](int64_t time, Imu imu){
        std::cout << "imu:" << time << std::endl;
    });

    data_.regist_callback_encoder([](int64_t time, Encoder encoder){
        std::cout << "encoder:" << time << std::endl;
    });

    data_.start();

    return  0;
}
