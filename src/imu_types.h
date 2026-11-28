#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t node_id;      // np. 0 = wysyłacz/lokalny, 1 = ESP_A, 2 = ESP_B
    uint32_t seq;
    uint32_t timestamp_ms;
    float qw;             // kwaternion orientacji (po wyzerowaniu montażu)
    float qx;
    float qy;
    float qz;
    float roll;           // [deg]
    float pitch;          // [deg]
    float yaw;            // [deg]
} imu_sample_t;