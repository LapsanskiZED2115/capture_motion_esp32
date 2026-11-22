// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c_master.h"
// #include "driver/gpio.h"

//address rejestru w ktorym znajduje sie bit sleep
//#define REGISTER_POWER_MANAGEMENT 0x6B
//#define REGISTER_DATA_START 0x3B

//#define NUMBER_OF_CALIBRATION_SAMPLES 200

// void app_main(void)
// {
//     //Magistrala I2C
//     i2c_master_bus_handle_t i2c_bus_handle;
//     i2c_master_bus_config_t i2c_bus_config = {
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .i2c_port = I2C_NUM_0,
//         .sda_io_num = 21,
//         .scl_io_num = 22,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };
//     i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
//
//     i2c_master_dev_handle_t mpu6050_handle;
//     i2c_device_config_t mpu6050_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = 0x68,
//         .scl_speed_hz = 400000,
//     };
//     i2c_master_bus_add_device(i2c_bus_handle, &mpu6050_config, &mpu6050_handle);
//
//     // Wybudzenie czujnika
//     uint8_t wake_up_message[2] = {REGISTER_POWER_MANAGEMENT, 0x00};
//     i2c_master_transmit(mpu6050_handle, wake_up_message, 2, 1000);
//     vTaskDelay(pdMS_TO_TICKS(100));
//
//     // Przycisk kalibracji
//     // 0 to przycisk wcisniety 1 to przycisk nie wcisniety
//     gpio_set_direction(0, GPIO_MODE_INPUT);
//     gpio_set_pull_mode(0, GPIO_PULLUP_ONLY);
//     int previous_button_level = 1;
//
//     uint8_t register_to_read = REGISTER_DATA_START;
//     // od rejestru 0x3B do 0x48 jest 14 bajtow danych
//     uint8_t received_bytes[14];
//
//     // Bias zyroskopu
//     float gyroscope_x_bias_raw = 0.0f;
//     float gyroscope_y_bias_raw = 0.0f;
//     float gyroscope_z_bias_raw = 0.0f;
//
//     // Flaga do kalibracji
//     int calibration_requested = 1;
//
//     while (1)
//     {
//         // jezeli przycisk wcisniety
//         int current_button_level = gpio_get_level(0);
//         if (previous_button_level == 1 && current_button_level == 0)
//         {
//             calibration_requested = 1;
//         }
//         previous_button_level = current_button_level;
//
//         // Kalibracja zera zyroskopu
//         if (calibration_requested == 1)
//         {
//             printf("Kalibracja \n");
//             float gyroscope_x_sum_raw = 0.0f;
//             float gyroscope_y_sum_raw = 0.0f;
//             float gyroscope_z_sum_raw = 0.0f;
//
//             for (int sample_index = 0; sample_index < NUMBER_OF_CALIBRATION_SAMPLES; sample_index++)
//             {
//                 i2c_master_transmit_receive(
//                     mpu6050_handle, &register_to_read, 1, received_bytes, 14, 1000);
//                 // sklejamy MSB i LSB w 16 bitowa wartosc
//                 gyroscope_x_sum_raw += (int16_t)((received_bytes[8] << 8) | received_bytes[9]);
//                 gyroscope_y_sum_raw += (int16_t)((received_bytes[10] << 8) | received_bytes[11]);
//                 gyroscope_z_sum_raw += (int16_t)((received_bytes[12] << 8) | received_bytes[13]);
//
//                 vTaskDelay(pdMS_TO_TICKS(10));
//             }
//
//             gyroscope_x_bias_raw = gyroscope_x_sum_raw / NUMBER_OF_CALIBRATION_SAMPLES;
//             gyroscope_y_bias_raw = gyroscope_y_sum_raw / NUMBER_OF_CALIBRATION_SAMPLES;
//             gyroscope_z_bias_raw = gyroscope_z_sum_raw / NUMBER_OF_CALIBRATION_SAMPLES;
//
//             previous_button_level = gpio_get_level(0);
//
//             calibration_requested = 0;
//             printf("Kalibracja Done \n");
//         }
//
//         esp_err_t result = i2c_master_transmit_receive(
//             mpu6050_handle, &register_to_read, 1, received_bytes, 14, 1000);
//
//         if (result == ESP_OK)
//         {   //mpu wysla wartosci w 2 bajtach MSB i LSB, sklejamy je w 16 bitowa wartosc
//             //przesuneicie bitowe jest zeby
//             int16_t accelerometer_x_raw = (int16_t)((received_bytes[0] << 8) | received_bytes[1]);
//             int16_t accelerometer_y_raw = (int16_t)((received_bytes[2] << 8) | received_bytes[3]);
//             int16_t accelerometer_z_raw = (int16_t)((received_bytes[4] << 8) | received_bytes[5]);
//
//             int16_t gyroscope_x_raw = (int16_t)((received_bytes[8] << 8) | received_bytes[9]);
//             int16_t gyroscope_y_raw = (int16_t)((received_bytes[10] << 8) | received_bytes[11]);
//             int16_t gyroscope_z_raw = (int16_t)((received_bytes[12] << 8) | received_bytes[13]);
//
//             // gyroscope_calibrated = gyroscope_raw - bias
//             float gyroscope_x_calibrated = gyroscope_x_raw - gyroscope_x_bias_raw;
//             float gyroscope_y_calibrated = gyroscope_y_raw - gyroscope_y_bias_raw;
//             float gyroscope_z_calibrated = gyroscope_z_raw - gyroscope_z_bias_raw;
//
//             printf("Accel %d %d %d / Gyro %d %d %d / Gyro  %.1f %.1f %.1f\n",
//                    accelerometer_x_raw, accelerometer_y_raw, accelerometer_z_raw,
//                    gyroscope_x_raw, gyroscope_y_raw, gyroscope_z_raw,
//                    gyroscope_x_calibrated, gyroscope_y_calibrated, gyroscope_z_calibrated);
//         }
//         else
//         {
//             printf("Blad");
//         }
//
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

static const char *TAG = "SPP_MPU_DEMO";
static const char *BT_DEVICE_NAME = "ESP32_MPU6050_SPP";
static const char *SPP_SERVER_NAME = "SPP_MPU_STREAM";

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    uint32_t seq;
} mpu6050_sample_t;

static volatile uint32_t s_spp_handle = 0;
static volatile bool s_spp_connected = false;

static esp_err_t mpu6050_mock_read(mpu6050_sample_t *out)
{
    static uint32_t counter = 0;
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float t = (float)counter * 0.1f;
    out->ax = 0.25f * sinf(t);
    out->ay = 0.25f * cosf(t);
    out->az = 1.00f + 0.05f * sinf(2.0f * t);

    out->gx = 15.0f * cosf(t);
    out->gy = 12.0f * sinf(0.5f * t);
    out->gz = 5.0f * sinf(t);
    out->seq = counter++;

    return ESP_OK;
}

static esp_err_t spp_send_line(const char *line)
{
    if (line == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_spp_connected || s_spp_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_spp_write(s_spp_handle, strlen(line), (uint8_t *)line);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_spp_write error: %s", esp_err_to_name(err));
    }
    return err;
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Parowanie udane: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGW(TAG, "Parowanie nieudane, status: %d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP Numeric Comparison: %lu", (unsigned long)param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "SSP Passkey: %lu", (unsigned long)param->key_notif.passkey);
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGD(TAG, "GAP mode: %d", param->mode_chg.mode);
        break;

    default:
        break;
    }
}

static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "SPP zainicjalizowane");
            esp_bt_gap_set_device_name(BT_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(TAG, "Błąd inicjalizacji SPP: %d", param->init.status);
        }
        break;

    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "Serwer SPP uruchomiony. Sparuj urządzenie z: %s", BT_DEVICE_NAME);
        } else {
            ESP_LOGE(TAG, "Błąd startu serwera SPP: %d", param->start.status);
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        s_spp_handle = param->srv_open.handle;
        s_spp_connected = true;
        ESP_LOGI(TAG, "Klient połączony (handle=%lu)", (unsigned long)s_spp_handle);
        break;

    case ESP_SPP_CLOSE_EVT:
        s_spp_connected = false;
        s_spp_handle = 0;
        ESP_LOGI(TAG, "Klient rozłączony");
        break;

    case ESP_SPP_WRITE_EVT:
        if (param->write.status != ESP_SPP_SUCCESS) {
            ESP_LOGW(TAG, "Błąd zapisu SPP: %d", param->write.status);
        }
        break;

    case ESP_SPP_DATA_IND_EVT:
        /* Jeśli chcesz odbierać komendy z telefonu/PC, obsłuż je tutaj */
        ESP_LOGD(TAG, "Odebrano %d bajtów", param->data_ind.len);
        break;

    default:
        break;
    }
}

static esp_err_t spp_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_gap_register_callback(bt_gap_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_register_callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_spp_register_callback(spp_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_spp_register_callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
    };
    ret = esp_spp_enhanced_init(&spp_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_spp_enhanced_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    return ESP_OK;
}

static void spp_mpu_stream_task(void *pvParameters)
{
    (void)pvParameters;
    char payload[160];
    mpu6050_sample_t sample;

    while (true) {
        if (s_spp_connected) {
            if (mpu6050_mock_read(&sample) == ESP_OK) {
                int len = snprintf(
                    payload,
                    sizeof(payload),
                    "{\"seq\":%lu,\"acc\":[%.3f,%.3f,%.3f],\"gyro\":[%.2f,%.2f,%.2f]}\r\n",
                    (unsigned long)sample.seq,
                    sample.ax, sample.ay, sample.az,
                    sample.gx, sample.gy, sample.gz
                );
                if (len > 0 && len < (int)sizeof(payload)) {
                    spp_send_line(payload);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(spp_init());

    xTaskCreate(
        spp_mpu_stream_task,
        "spp_mpu_task",
        4096,
        NULL,
        5,
        NULL
    );
}
