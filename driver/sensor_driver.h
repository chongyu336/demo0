#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

typedef struct {
    uint32_t timestamp_ms;
    float gyr_radDs[3];
    float acc_mDs2[3];
} imu_data_t;

typedef struct {
    uint32_t timestamp_ms;
    float mag_gauss[3];
} mag_data_t;

typedef struct {
    uint32_t timestamp_ms;
    float temperature_deg;
    float pressure_pa;
    float depth_m;
    float dt_ms;
} bar_data_t;

typedef struct {
    uint32_t timestamp_ms;
    float distance_m;
} rnf_data_t;

void sensor_init();
void sensor_collect();
#endif // SENSOR_DRIVER_H