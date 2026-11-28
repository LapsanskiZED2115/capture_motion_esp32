#pragma once

#include "esp_err.h"
#include "imu_types.h"

esp_err_t mpu6050_imu_init(uint8_t node_id);
/* Jeden krok: odczyt MPU6050 + aktualizacja filtra Mahony'ego (wołać jak najczęściej) */
esp_err_t mpu6050_imu_update(void);
/* Zrzut aktualnej orientacji do próbki */
esp_err_t mpu6050_imu_read(imu_sample_t *out);