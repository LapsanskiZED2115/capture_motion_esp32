#pragma once

#include "esp_err.h"
#include "imu_types.h"

esp_err_t mpu6050_imu_init(uint8_t node_id);
esp_err_t mpu6050_imu_read(imu_sample_t *out);

esp_err_t mpu6050_imu_calibrate_gyroscope(void);