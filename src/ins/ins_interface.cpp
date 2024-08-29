#include "ins_interface.h"


extern imu_data_t imu_data;
extern mag_data_t mag_data;
extern bar_data_t bar_data;
extern rnf_data_t rnf_data;

IMU_Bus imu_bus;
MAG_Bus mag_bus;
Bar_Bus bar_bus;
Rnf_Bus rnf_bus;

INS_Bus ins_bus;

// TimeTag ins_interval ={
//     .tag = 0,
//     .period = 100,
// };

void ins_interface_init()
{
    sensor_init();

}

uint32_t last_timestamp;
void ins_interface_step(uint32_t timestamp)
{
    //这里主要就是复制数据，然后调用滤波函数
    imu_bus.timestamp = timestamp;
    imu_bus.gyr_x = imu_data.gyr_radDs[0];
    imu_bus.gyr_y = imu_data.gyr_radDs[1];
    imu_bus.gyr_z = imu_data.gyr_radDs[2];
    imu_bus.acc_x = imu_data.acc_mDs2[0];
    imu_bus.acc_y = imu_data.acc_mDs2[1];
    imu_bus.acc_z = imu_data.acc_mDs2[2];

    bool clipping[3] = { false, false, false };
    uint32_t dt_imu = (timestamp >= last_timestamp) ? (timestamp - last_timestamp) : (0xFFFFFFFF - last_timestamp + timestamp);
    last_timestamp = timestamp;
    //******* *找一下开源的滤波算法
    Ekf_IMU_update();

    mag_bus.timestamp = timestamp;
    mag_bus.mag_x = mag_data.mag_gauss[0];
    mag_bus.mag_y = mag_data.mag_gauss[1];
    mag_bus.mag_z = mag_data.mag_gauss[2];
    //******* *//
    Ekf_MAG_update();

    bar_bus.timestamp = timestamp;
    bar_bus.pressure = bar_data.pressure_pa;
    bar_bus.temperature = bar_data.temperature_deg;
    bar_bus.depth = bar_data.depth_m;
    bar_bus.dt = bar_data.dt_ms;
    //******* *//
    Ekf_BAR_update();

    rnf_bus.timestamp = timestamp;
    rnf_bus.distance = rnf_data.distance_m;

    //ins_bus这个结构体就是另外两个模块拿来用的
    ins_bus.bar = &bar_bus;
    ins_bus.imu = &imu_bus;
    ins_bus.mag = &mag_bus;
    ins_bus.rnf = &rnf_bus;
    

}