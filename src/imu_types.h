#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t node_id;      // np. 0 = wysyłacz/lokalny, 1 = ESP_A, 2 = ESP_B
    uint32_t seq;
    uint32_t timestamp_ms;
    float ax;             // [g]
    float ay;             // [g]
    float az;             // [g]
    float gx;             // [deg/s]
    float gy;             // [deg/s]
    float gz;             // [deg/s]
} imu_sample_t;
