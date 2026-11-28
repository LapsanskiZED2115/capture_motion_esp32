#include "bluetooth_spp.h"
#include <stdio.h>
#include <string.h>

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

static const char *TAG = "BT_SPP";

static volatile uint32_t s_spp_handle = 0;
static volatile bool s_spp_connected = false;
static char s_device_name[32] = "ESP32_SPP_GATEWAY";
static char s_service_name[32] = "SPP_IMU_STREAM";

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
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

    default:
        break;
    }
}

static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            esp_bt_gap_set_device_name(s_device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, s_service_name);
        } else {
            ESP_LOGE(TAG, "Błąd inicjalizacji SPP: %d", param->init.status);
        }
        break;

    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "Serwer SPP gotowy (%s)", s_device_name);
        } else {
            ESP_LOGE(TAG, "Błąd startu serwera SPP: %d", param->start.status);
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        s_spp_handle = param->srv_open.handle;
        s_spp_connected = true;
        ESP_LOGI(TAG, "Host połączony (handle=%lu)", (unsigned long)s_spp_handle);
        break;

    case ESP_SPP_CLOSE_EVT:
        s_spp_connected = false;
        s_spp_handle = 0;
        ESP_LOGI(TAG, "Host rozłączony");
        break;

    case ESP_SPP_WRITE_EVT:
        if (param->write.status != ESP_SPP_SUCCESS) {
            ESP_LOGW(TAG, "Błąd zapisu SPP: %d", param->write.status);
        }
        break;

    default:
        break;
    }
}

esp_err_t bt_spp_init(const bt_spp_config_t *cfg)
{
    if (cfg != NULL) {
        if (cfg->device_name != NULL) {
            snprintf(s_device_name, sizeof(s_device_name), "%s", cfg->device_name);
        }
        if (cfg->service_name != NULL) {
            snprintf(s_service_name, sizeof(s_service_name), "%s", cfg->service_name);
        }
    }

    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) {
        return ret;
    }

    /* Uwaga: jeśli w przyszłości na tym samym ESP32 dołożysz ESP-NOW (Wi-Fi),
     * nie zwalniaj pamięci BLE/Wi-Fi w sposób kolidujący z trybem Dual Mode. */
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_bt_controller_mem_release info: %s", esp_err_to_name(ret));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_bt_gap_register_callback(bt_gap_cb);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_spp_register_callback(spp_cb);
    if (ret != ESP_OK) {
        return ret;
    }

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
    };
    ret = esp_spp_enhanced_init(&spp_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    return ESP_OK;
}

bool bt_spp_is_connected(void)
{
    return s_spp_connected && (s_spp_handle != 0);
}

esp_err_t bt_spp_send_bytes(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!bt_spp_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_spp_write(s_spp_handle, (int)len, (uint8_t *)data);
}

esp_err_t bt_spp_send_sample_json(const imu_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!bt_spp_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    char line[192];
    int len = snprintf(
        line,
        sizeof(line),
        "{\"node\":%u,\"seq\":%lu,\"ts\":%lu,\"q\":[%.4f,%.4f,%.4f,%.4f],\"euler\":[%.1f,%.1f,%.1f]}\r\n",
        (unsigned)sample->node_id,
        (unsigned long)sample->seq,
        (unsigned long)sample->timestamp_ms,
        sample->qw, sample->qx, sample->qy, sample->qz,
        sample->roll, sample->pitch, sample->yaw
    );
    if (len <= 0 || len >= (int)sizeof(line)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return bt_spp_send_bytes((const uint8_t *)line, (size_t)len);
}
