#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "imu_types.h"

typedef struct {
    const char *device_name;
    const char *service_name;
} bt_spp_config_t;

esp_err_t bt_spp_init(const bt_spp_config_t *cfg);
bool bt_spp_is_connected(void);
esp_err_t bt_spp_send_bytes(const uint8_t *data, size_t len);
esp_err_t bt_spp_send_sample_json(const imu_sample_t *sample);
