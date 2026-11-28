

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"

#include "imu_types.h"
#include "mpu6050_imu.h"
#include "bluetooth_spp.h"
#include "link_rx.h"

#include "device_mac.h"

static const char *TAG = "MAIN";

#define LOCAL_NODE_ID       0U
#define SAMPLE_QUEUE_LEN    32U
#define SAMPLE_PERIOD_MS    100U

static QueueHandle_t s_tx_queue = NULL;


static void imu_producer_task(void *pvParameters)
{
    (void)pvParameters;
    imu_sample_t sample;

    while (true) {
        if (mpu6050_imu_read(&sample) == ESP_OK) {

       
            //Tu mozesz wstawic wywloanie mahonego 

//            if (xQueueSend(s_tx_queue, &sample, 0) != pdTRUE)  jako sample maja wleciec wyniki z mahonego do tej
//linijki nizej zeby wyslac te wartosci na bluetootha
            if (xQueueSend(s_tx_queue, &sample, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Kolejka nadawcza pełna, pominięto próbkę seq=%lu", (unsigned long)sample.seq);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}


 



static void spp_sender_task(void *pvParameters)
{
    (void)pvParameters;
    imu_sample_t sample;

    while (true) {
        if (xQueueReceive(s_tx_queue, &sample, portMAX_DELAY) == pdTRUE) {
            if (bt_spp_is_connected()) {
                bt_spp_send_sample_json(&sample);
            }
        }
    }
}

void app_main(void)
{
    s_tx_queue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(imu_sample_t));
    configASSERT(s_tx_queue != NULL);

    ESP_ERROR_CHECK(mpu6050_imu_init(LOCAL_NODE_ID));

    bt_spp_config_t spp_cfg = {
        .device_name = "ESP32_MPU6050_SPP",
        .service_name = "SPP_IMU_STREAM",
    };
    ESP_ERROR_CHECK(bt_spp_init(&spp_cfg));
    ESP_ERROR_CHECK(link_rx_init(s_tx_queue));
    device_mac_start_serial_command_listener();

    xTaskCreate(imu_producer_task, "imu_prod", 4096, NULL, 5, NULL);
    xTaskCreate(spp_sender_task, "spp_send", 4096, NULL, 6, NULL);
}
