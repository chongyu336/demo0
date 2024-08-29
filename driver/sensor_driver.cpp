#include "sensor_driver.h"
#include "systim.h"
#include "filter.h"

imu_data_t imu_data;
mag_data_t mag_data;
bar_data_t bar_data;
rnf_data_t rnf_data;

TimeTag imu_interval = {
    .tag = 0,
    .period = 1,
};
TimeTag mag_interval ={
    .tag = 0,
    .period = 10,
};
TimeTag bar_interval ={
    .tag = 0,
    .period = 10,
};

TimeTag rnf_interval ={
    .tag = 0,
    .period = 10,
};


void sensor_init()
{
    //传感器初始化：需要相应函数
    imu_init();
    mag_init();
    bar_init();
    rnf_init();
}


/**
 * @brief Collect sensor data
 * @note Should be invoked periodically. e.g, at 1KHz
 */
void sensor_collect()
{
    //temp用来存放临时数据
    float         temp[3];
    float         temp1[3];
    if (check_timetag(&imu_interval)) //判断是否到达采样时间
    {
        imu_data.timestamp_ms = systime_now_ms();
        if() //里面是读数据的功能，并判断是否读成功，注意数据的顺序
        {
            
        /************************************ */
            imu_data.gyr_radDs[0] = temp[0];
            imu_data.gyr_radDs[1] = temp[1];
            imu_data.gyr_radDs[2] = temp[2];
            /************************************ */
            imu_data.acc_mDs2[0] = temp1[0];
            imu_data.acc_mDs2[1] = temp1[1];
            imu_data.acc_mDs2[2] = temp1[2];
        }
    }
    if(check_timetag(&mag_interval))
    {
        mag_data.timestamp_ms = systime_now_ms();
        if()
        {
            /************************************ */
            mag_data.mag_gauss[0] = temp[0];
            mag_data.mag_gauss[1] = temp[1];
            mag_data.mag_gauss[2] = temp[2];
            /************************************ */
        }
    }

    if(check_timetag(&bar_interval))
    {
        bar_data.dt_ms = systime_now_ms() - bar_data.timestamp_ms;
        bar_data.timestamp_ms = systime_now_ms();
        if()
        {
        /************************************ */
            bar_data.temperature_deg = temp[0];
            bar_data.pressure_pa = temp[1];
            bar_data.depth_m = temp[2];
        /************************************ */
        }
    }
    if(check_timetag(&rnf_interval))
    {
        rnf_data.timestamp_ms = systime_now_ms();
        if()
        {
        /************************************ */
            rnf_data.distance_m = temp[0];
        /************************************ */
        }
    }
}