#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

//address rejestru w ktorym znajduje sie bit sleep
#define REGISTER_POWER_MANAGEMENT 0x6B
#define REGISTER_DATA_START 0x3B

#define NUMBER_OF_CALIBRATION_SAMPLES 200

void app_main(void)
{
    //Magistrala I2C
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);

    i2c_master_dev_handle_t mpu6050_handle;
    i2c_device_config_t mpu6050_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x68,
        .scl_speed_hz = 400000,
    };
    i2c_master_bus_add_device(i2c_bus_handle, &mpu6050_config, &mpu6050_handle);

    // Wybudzenie czujnika
    uint8_t wake_up_message[2] = {REGISTER_POWER_MANAGEMENT, 0x00};
    i2c_master_transmit(mpu6050_handle, wake_up_message, 2, 1000);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Przycisk kalibracji
    // 0 to przycisk wcisniety 1 to przycisk nie wcisniety
    gpio_set_direction(0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(0, GPIO_PULLUP_ONLY);
    int previous_button_level = 1;

    uint8_t register_to_read = REGISTER_DATA_START;
    // od rejestru 0x3B do 0x48 jest 14 bajtow danych
    uint8_t received_bytes[14];

    // Bias zyroskopu
    float gyroscope_x_bias_raw = 0.0f;
    float gyroscope_y_bias_raw = 0.0f;
    float gyroscope_z_bias_raw = 0.0f;

    // Flaga do kalibracji
    int calibration_requested = 1;

    while (1)
    {
        // jezeli przycisk wcisniety
        int current_button_level = gpio_get_level(0);
        if (previous_button_level == 1 && current_button_level == 0)
        {
            calibration_requested = 1;
        }
        previous_button_level = current_button_level;

        // Kalibracja zera zyroskopu
        if (calibration_requested == 1)
        {
            printf("Kalibracja \n");
            float gyroscope_x_sum_raw = 0.0f;
            float gyroscope_y_sum_raw = 0.0f;
            float gyroscope_z_sum_raw = 0.0f;

            for (int sample_index = 0; sample_index < NUMBER_OF_CALIBRATION_SAMPLES; sample_index++)
            {
                i2c_master_transmit_receive(
                    mpu6050_handle, &register_to_read, 1, received_bytes, 14, 1000);
                // sklejamy MSB i LSB w 16 bitowa wartosc
                gyroscope_x_sum_raw += (int16_t)((received_bytes[8] << 8) | received_bytes[9]);
                gyroscope_y_sum_raw += (int16_t)((received_bytes[10] << 8) | received_bytes[11]);
                gyroscope_z_sum_raw += (int16_t)((received_bytes[12] << 8) | received_bytes[13]);

                vTaskDelay(pdMS_TO_TICKS(10));
            }

            gyroscope_x_bias_raw = gyroscope_x_sum_raw / NUMBER_OF_CALIBRATION_SAMPLES;
            gyroscope_y_bias_raw = gyroscope_y_sum_raw / NUMBER_OF_CALIBRATION_SAMPLES;
            gyroscope_z_bias_raw = gyroscope_z_sum_raw / NUMBER_OF_CALIBRATION_SAMPLES;

            previous_button_level = gpio_get_level(0);

            calibration_requested = 0;
            printf("Kalibracja Done \n");
        }

        esp_err_t result = i2c_master_transmit_receive(
            mpu6050_handle, &register_to_read, 1, received_bytes, 14, 1000);

        if (result == ESP_OK)
        {   //mpu wysla wartosci w 2 bajtach MSB i LSB, sklejamy je w 16 bitowa wartosc
            //przesuneicie bitowe jest zeby 
            int16_t accelerometer_x_raw = (int16_t)((received_bytes[0] << 8) | received_bytes[1]);
            int16_t accelerometer_y_raw = (int16_t)((received_bytes[2] << 8) | received_bytes[3]);
            int16_t accelerometer_z_raw = (int16_t)((received_bytes[4] << 8) | received_bytes[5]);

            int16_t gyroscope_x_raw = (int16_t)((received_bytes[8] << 8) | received_bytes[9]);
            int16_t gyroscope_y_raw = (int16_t)((received_bytes[10] << 8) | received_bytes[11]);
            int16_t gyroscope_z_raw = (int16_t)((received_bytes[12] << 8) | received_bytes[13]);

            // gyroscope_calibrated = gyroscope_raw - bias
            float gyroscope_x_calibrated = gyroscope_x_raw - gyroscope_x_bias_raw;
            float gyroscope_y_calibrated = gyroscope_y_raw - gyroscope_y_bias_raw;
            float gyroscope_z_calibrated = gyroscope_z_raw - gyroscope_z_bias_raw;

            printf("Accel %d %d %d / Gyro %d %d %d / Gyro  %.1f %.1f %.1f\n",
                   accelerometer_x_raw, accelerometer_y_raw, accelerometer_z_raw,
                   gyroscope_x_raw, gyroscope_y_raw, gyroscope_z_raw,
                   gyroscope_x_calibrated, gyroscope_y_calibrated, gyroscope_z_calibrated);
        }
        else
        {
            printf("Blad");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}