#include "device_mac.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define COMMAND_BUFFER_SIZE 32

static const char *TAG = "DEVICE_MAC";

void device_mac_print(void)
{
    uint8_t bluetooth_mac_address[6];

    esp_read_mac(bluetooth_mac_address, ESP_MAC_BT);

    printf("MAC Bluetooth:       %02X:%02X:%02X:%02X:%02X:%02X\n",
           bluetooth_mac_address[0], bluetooth_mac_address[1], bluetooth_mac_address[2],
           bluetooth_mac_address[3], bluetooth_mac_address[4], bluetooth_mac_address[5]);
}

static void serial_command_task(void *pvParameters)
{
    (void)pvParameters;

    char command_buffer[COMMAND_BUFFER_SIZE];
    int number_of_received_characters = 0;

    while (true)
    {
        uint8_t received_character;
        int number_of_read_bytes = uart_read_bytes(UART_NUM_0, &received_character, 1, portMAX_DELAY);

        if (number_of_read_bytes != 1)
        {
            continue;
        }

        if (received_character == '\n' || received_character == '\r')
        {
            command_buffer[number_of_received_characters] = '\0';

            if (strcmp(command_buffer, "mac") == 0)
            {
                device_mac_print();
            }

            number_of_received_characters = 0;
        }
        else if (number_of_received_characters < COMMAND_BUFFER_SIZE - 1)
        {
            command_buffer[number_of_received_characters] = (char)received_character;
            number_of_received_characters++;
        }
    }
}

void device_mac_start_serial_command_listener(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    xTaskCreate(serial_command_task, "serial_cmd", 3072, NULL, 3, NULL);
    ESP_LOGI(TAG, "Wpisz 'mac' + Enter, aby wypisac adresy MAC");
}