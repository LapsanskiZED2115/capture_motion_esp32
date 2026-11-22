#include "link_rx.h"
#include "esp_log.h"

static const char *TAG = "LINK_RX";
static QueueHandle_t s_tx_queue = NULL;

esp_err_t link_rx_init(QueueHandle_t tx_queue)
{
    if (tx_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_tx_queue = tx_queue;

    /* Tu w kolejnym etapie podepniesz np. ESP-NOW recv callback lub task UART/SPI,
     * który sparsuje odebrany pakiet do imu_sample_t i zrobi:
     * xQueueSend(s_tx_queue, &remote_sample, 0); */
    ESP_LOGI(TAG, "Moduł odbioru z innych ESP32 gotowy pod podpięcie transportu");
    return ESP_OK;
}
