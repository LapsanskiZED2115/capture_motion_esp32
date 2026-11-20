#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// Rejestry MPU6050
#define REGISTER_CONFIG           0x1A
#define REGISTER_GYRO_CONFIG      0x1B
#define REGISTER_ACCEL_CONFIG     0x1C
#define REGISTER_DATA_START       0x3B
#define REGISTER_POWER_MANAGEMENT 0x6B

#define NUMBER_OF_CALIBRATION_SAMPLES 200

// Czulosci dla zakresow +-16g (0x18) oraz +-2000 deg/s (0x18)
#define ACCEL_SENSITIVITY 2048.0f
#define GYRO_SENSITIVITY  16.4f

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define MAHONY_KP  2.5f
#define MAHONY_KI  0.05f

typedef struct { float w, x, y, z; } quaternion_t;
typedef struct { float x, y, z; } vec3_t;

typedef struct {
    uint8_t id;
    vec3_t gyro_bias_dps;
    vec3_t prev_gyro_rad;    // poprzednia predkosc katowa
    vec3_t integral_fb;      // calka bledu Mahony'ego

    quaternion_t q;          // surowa orientacja czujnika
    quaternion_t q_cal_inv;  // odwrotnosc orientacji z momentu kalibracji
    quaternion_t q_segment;  // przetworzona orientacja po wyzerowaniu montazu
} imu_state_t;

// kwaterniony

static quaternion_t quat_multiply(quaternion_t a, quaternion_t b)
{
    return (quaternion_t){
        .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

static quaternion_t quat_conjugate(quaternion_t q)
{
    return (quaternion_t){ q.w, -q.x, -q.y, -q.z };
}

static void quat_to_euler_deg(quaternion_t q, float* roll, float* pitch, float* yaw)
{
    *roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    *pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * RAD_TO_DEG;
    *yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * RAD_TO_DEG;
}

// kalibracja+filtr

void imu_calibrate_from_sums(imu_state_t* imu, uint8_t id, vec3_t a_sum, vec3_t g_sum, int count)
{
    imu->id = id;

    imu->gyro_bias_dps.x = g_sum.x / count;
    imu->gyro_bias_dps.y = g_sum.y / count;
    imu->gyro_bias_dps.z = g_sum.z / count;

    imu->prev_gyro_rad = (vec3_t){ 0.0f, 0.0f, 0.0f };
    imu->integral_fb   = (vec3_t){ 0.0f, 0.0f, 0.0f };

    float ax = a_sum.x / count;
    float ay = a_sum.y / count;
    float az = a_sum.z / count;
    float norm = sqrtf(ax * ax + ay * ay + az * az);

    if (norm > 0.0001f)
    {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }
    else
    {
        ax = 0.0f; ay = 0.0f; az = 1.0f;
    }

    // obrot osi z do orientacji zmierzonej przez akcelerometr
    if (az < -0.9999f)
    {
        imu->q = (quaternion_t){ 0.0f, 1.0f, 0.0f, 0.0f };
    }
    else
    {
        float qw = 1.0f + az;
        float qx = ay;
        float qy = -ax;
        float qz = 0.0f;

        float q_norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
        imu->q.w = qw / q_norm;
        imu->q.x = qx / q_norm;
        imu->q.y = qy / q_norm;
        imu->q.z = qz / q_norm;
    }

    imu->q_cal_inv = quat_conjugate(imu->q);
    imu->q_segment = (quaternion_t){ 1.0f, 0.0f, 0.0f, 0.0f };
}

void imu_update(imu_state_t* imu, vec3_t accel_g, vec3_t gyro_dps, float dt)
{
    float gx_dps = gyro_dps.x - imu->gyro_bias_dps.x;
    float gy_dps = gyro_dps.y - imu->gyro_bias_dps.y;
    float gz_dps = gyro_dps.z - imu->gyro_bias_dps.z;

    // pozbycie się szumu w spoczynku
    if (fabsf(gx_dps) < 0.25f) gx_dps = 0.0f;
    if (fabsf(gy_dps) < 0.25f) gy_dps = 0.0f;
    if (fabsf(gz_dps) < 0.25f) gz_dps = 0.0f;

    float gx = gx_dps * DEG_TO_RAD;
    float gy = gy_dps * DEG_TO_RAD;
    float gz = gz_dps * DEG_TO_RAD;

    // calkowanie zyro
    float gx_avg = 0.5f * (imu->prev_gyro_rad.x + gx);
    float gy_avg = 0.5f * (imu->prev_gyro_rad.y + gy);
    float gz_avg = 0.5f * (imu->prev_gyro_rad.z + gz);

    imu->prev_gyro_rad.x = gx;
    imu->prev_gyro_rad.y = gy;
    imu->prev_gyro_rad.z = gz;

    float ax = accel_g.x, ay = accel_g.y, az = accel_g.z;
    float q0 = imu->q.w, q1 = imu->q.x, q2 = imu->q.y, q3 = imu->q.z;
    float norm = sqrtf(ax * ax + ay * ay + az * az);

    // wylaczenie korekcji akcelerometrem przy większych przyspieszeniach
    if (norm > 0.85f && norm < 1.15f)
    {
        ax /= norm; ay /= norm; az /= norm;

        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        //douczanie biasu
        imu->integral_fb.x += MAHONY_KI * ex * dt;
        imu->integral_fb.y += MAHONY_KI * ey * dt;
        imu->integral_fb.z += MAHONY_KI * ez * dt;

        gx_avg += MAHONY_KP * ex + imu->integral_fb.x;
        gy_avg += MAHONY_KP * ey + imu->integral_fb.y;
        gz_avg += MAHONY_KP * ez + imu->integral_fb.z;
    }

    float half_dt = 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx_avg - qc * gy_avg - q3 * gz_avg) * half_dt;
    q1 += ( qa * gx_avg + qc * gz_avg - q3 * gy_avg) * half_dt;
    q2 += ( qa * gy_avg - qb * gz_avg + q3 * gx_avg) * half_dt;
    q3 += ( qa * gz_avg + qb * gy_avg - qc * gx_avg) * half_dt;

    float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (q_norm > 0.0001f)
    {
        imu->q.w = q0 / q_norm;
        imu->q.x = q1 / q_norm;
        imu->q.y = q2 / q_norm;
        imu->q.z = q3 / q_norm;
    }
    else
    {
        imu->q = (quaternion_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    }

    imu->q_segment = quat_multiply(imu->q, imu->q_cal_inv);
}

// odczyty

static esp_err_t mpu6050_read_scaled(i2c_master_dev_handle_t handle, vec3_t* out_acc_g, vec3_t* out_gyro_dps)
{
    uint8_t reg = REGISTER_DATA_START;
    uint8_t rx[14];

    esp_err_t err = i2c_master_transmit_receive(handle, &reg, 1, rx, 14, 1000);
    if (err != ESP_OK) return err;

    int16_t ax_raw = (int16_t)((rx[0] << 8) | rx[1]);
    int16_t ay_raw = (int16_t)((rx[2] << 8) | rx[3]);
    int16_t az_raw = (int16_t)((rx[4] << 8) | rx[5]);

    int16_t gx_raw = (int16_t)((rx[8] << 8) | rx[9]);
    int16_t gy_raw = (int16_t)((rx[10] << 8) | rx[11]);
    int16_t gz_raw = (int16_t)((rx[12] << 8) | rx[13]);

    out_acc_g->x = ax_raw / ACCEL_SENSITIVITY;
    out_acc_g->y = ay_raw / ACCEL_SENSITIVITY;
    out_acc_g->z = az_raw / ACCEL_SENSITIVITY;

    out_gyro_dps->x = gx_raw / GYRO_SENSITIVITY;
    out_gyro_dps->y = gy_raw / GYRO_SENSITIVITY;
    out_gyro_dps->z = gz_raw / GYRO_SENSITIVITY;

    return ESP_OK;
}

// main

void app_main(void)
{
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle));

    i2c_master_dev_handle_t mpu6050_handle;
    i2c_device_config_t mpu6050_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x68,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &mpu6050_config, &mpu6050_handle));

    uint8_t wake_up_msg[2] = { REGISTER_POWER_MANAGEMENT, 0x01 };
    uint8_t dlpf_msg[2]    = { REGISTER_CONFIG, 0x03 };       // DLPF ~42 Hz
    uint8_t gyro_msg[2]    = { REGISTER_GYRO_CONFIG, 0x18 };  // +-2000 deg/s
    uint8_t accel_msg[2]   = { REGISTER_ACCEL_CONFIG, 0x18 }; // +-16g

    i2c_master_transmit(mpu6050_handle, wake_up_msg, 2, 1000);
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_master_transmit(mpu6050_handle, dlpf_msg, 2, 1000);
    i2c_master_transmit(mpu6050_handle, gyro_msg, 2, 1000);
    i2c_master_transmit(mpu6050_handle, accel_msg, 2, 1000);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_direction(0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(0, GPIO_PULLUP_ONLY);
    int previous_button_level = 1;

    imu_state_t imu = {
        .q = { 1.0f, 0.0f, 0.0f, 0.0f },
        .q_cal_inv = { 1.0f, 0.0f, 0.0f, 0.0f },
        .q_segment = { 1.0f, 0.0f, 0.0f, 0.0f }
    };
    int calibration_requested = 1;
    int64_t last_time_us = esp_timer_get_time();
    int print_divider = 0;

    

    while (1)
    {
        int current_button_level = gpio_get_level(0);
        if (previous_button_level == 1 && current_button_level == 0)
        {
            calibration_requested = 1;
        }
        previous_button_level = current_button_level;

        if (calibration_requested == 1)
        {
            printf("Kalibracja... \n");
            vec3_t a_sum = { 0 }, g_sum = { 0 };
            int valid_samples = 0;
            int attempts = 0;

            // zbieramy probki tylko wtedy, gdy czujnik nie drga
            while (valid_samples < NUMBER_OF_CALIBRATION_SAMPLES && attempts < 600)
            {
                vec3_t acc_sample, gyro_sample;
                if (mpu6050_read_scaled(mpu6050_handle, &acc_sample, &gyro_sample) == ESP_OK)
                {
                    float gyro_mag = sqrtf(gyro_sample.x * gyro_sample.x +
                                           gyro_sample.y * gyro_sample.y +
                                           gyro_sample.z * gyro_sample.z);
                    if (gyro_mag < 8.0f)
                    {
                        a_sum.x += acc_sample.x; a_sum.y += acc_sample.y; a_sum.z += acc_sample.z;
                        g_sum.x += gyro_sample.x; g_sum.y += gyro_sample.y; g_sum.z += gyro_sample.z;
                        valid_samples++;
                    }
                }
                attempts++;
                vTaskDelay(pdMS_TO_TICKS(5));
            }

            if (valid_samples > 0)
            {
                imu_calibrate_from_sums(&imu, 0, a_sum, g_sum, valid_samples);
                printf("Kalibracja zakonczona (Probek: %d | Bias: %.2f, %.2f, %.2f)\n",
                       valid_samples, imu.gyro_bias_dps.x, imu.gyro_bias_dps.y, imu.gyro_bias_dps.z);
            }
            else
            {
                printf("Blad kalibracji - brak odczytow I2C lub ciagly ruch!\n");
            }

            previous_button_level = gpio_get_level(0);
            calibration_requested = 0;
            last_time_us = esp_timer_get_time();
        }

        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_time_us) / 1000000.0f;
        last_time_us = now_us;

        // zabezpieczenie przed zbyt duzym dt
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

        vec3_t acc_g, gyro_dps;
        if (mpu6050_read_scaled(mpu6050_handle, &acc_g, &gyro_dps) == ESP_OK)
        {
            imu_update(&imu, acc_g, gyro_dps, dt);

            if (print_divider == 0)
            {
                float roll, pitch, yaw;
                quat_to_euler_deg(imu.q_segment, &roll, &pitch, &yaw);

                printf("R:%6.1f P:%6.1f Y:%6.1f | Q:[%5.2f, %5.2f, %5.2f, %5.2f] | AccNorm:%.2fg\n",
                       roll, pitch, yaw,
                       imu.q_segment.w, imu.q_segment.x, imu.q_segment.y, imu.q_segment.z,
                       sqrtf(acc_g.x * acc_g.x + acc_g.y * acc_g.y + acc_g.z * acc_g.z));
            }
            print_divider = (print_divider + 1) % 5;
        }
        else
        {
            printf("Blad odczytu I2C\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}