#include "mpu6050_imu.h"
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MPU6050_IMU";
static uint8_t s_node_id = 0;
static uint32_t s_seq = 0;

esp_err_t mpu6050_imu_init(uint8_t node_id)
{
    s_node_id = node_id;
    s_seq = 0;
    /* Docelowo tutaj: inicjalizacja magistrali I2C i konfiguracja rejestrów MPU6050 */
    ESP_LOGI(TAG, "Zainicjalizowano moduł IMU (mock) dla node_id=%u", (unsigned)s_node_id);
    return ESP_OK;
}

esp_err_t mpu6050_imu_read(imu_sample_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float t = (float)s_seq * 0.1f;
    float phase_offset = (float)s_node_id * 1.047f; // lekko różne przebiegi dla różnych węzłów

    out->node_id = s_node_id;
    out->seq = s_seq++;
    out->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    out->ax = 0.25f * sinf(t + phase_offset);
    out->ay = 0.25f * cosf(t + phase_offset);
    out->az = 1.00f + 0.05f * sinf(2.0f * t + phase_offset);

    out->gx = 15.0f * cosf(t + phase_offset);
    out->gy = 12.0f * sinf(0.5f * t + phase_offset);
    out->gz = 5.0f * sinf(t + phase_offset);

    return ESP_OK;
}
