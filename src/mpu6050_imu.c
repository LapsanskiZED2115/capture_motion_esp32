#include "mpu6050_imu.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#define MPU6050_I2C_ADDRESS                    0x68
#define REGISTER_POWER_MANAGEMENT              0x6B
#define REGISTER_DATA_START                    0x3B

#define I2C_SDA_PIN                            21
#define I2C_SCL_PIN                            22
#define I2C_TIMEOUT_MS                         1000

#define NUMBER_OF_CALIBRATION_SAMPLES          200

#define ACCELEROMETER_LSB_PER_G                16384.0f
#define GYROSCOPE_LSB_PER_DEGREE_PER_SECOND    131.0f

static const char *TAG = "MPU6050_IMU";

static uint8_t s_node_id = 0;
static uint32_t s_seq = 0;

static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_mpu6050_handle = NULL;

static float s_gyroscope_x_bias_raw = 0.0f;
static float s_gyroscope_y_bias_raw = 0.0f;
static float s_gyroscope_z_bias_raw = 0.0f;

esp_err_t mpu6050_imu_calibrate_gyroscope(void)
{
    ESP_LOGI(TAG, "Kalibracja zyroskopu - nie ruszaj czujnika");

    uint8_t register_to_read = REGISTER_DATA_START;
    uint8_t received_bytes[14];

    float gyroscope_x_sum_raw = 0.0f;
    float gyroscope_y_sum_raw = 0.0f;
    float gyroscope_z_sum_raw = 0.0f;
    int number_of_good_samples = 0;

    for (int sample_index = 0; sample_index < NUMBER_OF_CALIBRATION_SAMPLES; sample_index++)
    {
        esp_err_t result = i2c_master_transmit_receive(
            s_mpu6050_handle, &register_to_read, 1, received_bytes, 14, I2C_TIMEOUT_MS);

        if (result == ESP_OK)
        {
            gyroscope_x_sum_raw += (int16_t)((received_bytes[8] << 8) | received_bytes[9]);
            gyroscope_y_sum_raw += (int16_t)((received_bytes[10] << 8) | received_bytes[11]);
            gyroscope_z_sum_raw += (int16_t)((received_bytes[12] << 8) | received_bytes[13]);
            number_of_good_samples++;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (number_of_good_samples == 0)
    {
        ESP_LOGE(TAG, "Kalibracja nieudana - brak poprawnych odczytow");
        return ESP_FAIL;
    }

    s_gyroscope_x_bias_raw = gyroscope_x_sum_raw / number_of_good_samples;
    s_gyroscope_y_bias_raw = gyroscope_y_sum_raw / number_of_good_samples;
    s_gyroscope_z_bias_raw = gyroscope_z_sum_raw / number_of_good_samples;

    ESP_LOGI(TAG, "Kalibracja zakonczona");
    return ESP_OK;
}

esp_err_t mpu6050_imu_init(uint8_t node_id)
{
    s_node_id = node_id;
    s_seq = 0;

    /* Magistrala I2C */
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t result = i2c_new_master_bus(&i2c_bus_config, &s_i2c_bus_handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Blad tworzenia magistrali I2C: %s", esp_err_to_name(result));
        return result;
    }

    /* Czujnik MPU6050 na magistrali */
    i2c_device_config_t mpu6050_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDRESS,
        .scl_speed_hz = 400000,
    };
    result = i2c_master_bus_add_device(s_i2c_bus_handle, &mpu6050_config, &s_mpu6050_handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Blad dodawania MPU6050: %s", esp_err_to_name(result));
        return result;
    }

    /* Wybudzenie czujnika (zerujemy bit sleep) */
    uint8_t wake_up_message[2] = {REGISTER_POWER_MANAGEMENT, 0x00};
    result = i2c_master_transmit(s_mpu6050_handle, wake_up_message, 2, I2C_TIMEOUT_MS);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Blad wybudzania MPU6050: %s", esp_err_to_name(result));
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Zainicjalizowano MPU6050 dla node_id=%u", (unsigned)s_node_id);

    return mpu6050_imu_calibrate_gyroscope();
}

esp_err_t mpu6050_imu_read(imu_sample_t *out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t register_to_read = REGISTER_DATA_START;
    uint8_t received_bytes[14];

    esp_err_t result = i2c_master_transmit_receive(
        s_mpu6050_handle, &register_to_read, 1, received_bytes, 14, I2C_TIMEOUT_MS);
    if (result != ESP_OK)
    {
        return result;
    }

    /* MPU wysyla kazda wartosc w 2 bajtach (MSB, LSB) - sklejamy w 16 bitow ze znakiem */
    int16_t accelerometer_x_raw = (int16_t)((received_bytes[0] << 8) | received_bytes[1]);
    int16_t accelerometer_y_raw = (int16_t)((received_bytes[2] << 8) | received_bytes[3]);
    int16_t accelerometer_z_raw = (int16_t)((received_bytes[4] << 8) | received_bytes[5]);

    /* bajty 6 i 7 to temperatura - pomijamy */
    int16_t gyroscope_x_raw = (int16_t)((received_bytes[8] << 8) | received_bytes[9]);
    int16_t gyroscope_y_raw = (int16_t)((received_bytes[10] << 8) | received_bytes[11]);
    int16_t gyroscope_z_raw = (int16_t)((received_bytes[12] << 8) | received_bytes[13]);

    out->node_id = s_node_id;
    out->seq = s_seq++;
    out->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    /* acceleration_in_g = accelerometer_raw / LSB_per_g */
    out->ax = accelerometer_x_raw / ACCELEROMETER_LSB_PER_G;
    out->ay = accelerometer_y_raw / ACCELEROMETER_LSB_PER_G;
    out->az = accelerometer_z_raw / ACCELEROMETER_LSB_PER_G;

    /* rotation_in_degrees_per_second = (gyroscope_raw - bias) / LSB_per_dps */
    out->gx = (gyroscope_x_raw - s_gyroscope_x_bias_raw) / GYROSCOPE_LSB_PER_DEGREE_PER_SECOND;
    out->gy = (gyroscope_y_raw - s_gyroscope_y_bias_raw) / GYROSCOPE_LSB_PER_DEGREE_PER_SECOND;
    out->gz = (gyroscope_z_raw - s_gyroscope_z_bias_raw) / GYROSCOPE_LSB_PER_DEGREE_PER_SECOND;

    return ESP_OK;
}