#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "imu_types.h"

/* Wywoływane na węźle-wysyłaczu, aby odbierać ramki z innych ESP32
 * i wrzucać je do tej samej kolejki co próbki lokalne. */
esp_err_t link_rx_init(QueueHandle_t tx_queue);
