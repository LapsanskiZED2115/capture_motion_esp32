#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define MAHONY_KP  2.5f

// Dlugosci segmentow w metrach
#define THIGH_LEN 0.42f // Udo
#define SHIN_LEN  0.40f // Piszczel
#define FOOT_LEN  0.22f // Stopa

typedef struct { float w, x, y, z; } quaternion_t;
typedef struct { float x, y, z; } vec3_t;

// Stan pojedynczego IMU
typedef struct {
    uint8_t id;
    vec3_t gyro_bias_dps;

    quaternion_t q;          // Surowa orientacja czujnika
    quaternion_t q_cal_inv;  // Odwrotnosc orientacji z momentu kalibracji
    quaternion_t q_segment;  // Przetworzona orientacja w przestrzeni
} imu_state_t;


static quaternion_t quat_multiply(quaternion_t a, quaternion_t b)
{
    return (quaternion_t) {
        .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

static quaternion_t quat_conjugate(quaternion_t q)
{
    return (quaternion_t) { q.w, -q.x, -q.y, -q.z };
}

static vec3_t quat_rotate_vec(quaternion_t q, vec3_t v)
{
    quaternion_t p = { 0.0f, v.x, v.y, v.z };
    quaternion_t res = quat_multiply(quat_multiply(q, p), quat_conjugate(q));
    return (vec3_t) { res.x, res.y, res.z };
}

static quaternion_t quat_from_axis_angle(vec3_t axis, float angle_rad)
{
    float half = angle_rad * 0.5f;
    float s = sinf(half);
    return (quaternion_t) { cosf(half), axis.x* s, axis.y* s, axis.z* s };
}

static void quat_to_euler_deg(quaternion_t q, float* roll, float* pitch, float* yaw)
{
    *roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    *pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * RAD_TO_DEG;
    *yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * RAD_TO_DEG;
}


void imu_calibrate_from_samples(imu_state_t* imu, uint8_t id, const vec3_t* acc_samples, const vec3_t* gyro_samples, int count)
{
    imu->id = id;
    vec3_t a_sum = { 0 }, g_sum = { 0 };

    for (int i = 0; i < count; i++) {
        a_sum.x += acc_samples[i].x; a_sum.y += acc_samples[i].y; a_sum.z += acc_samples[i].z;
        g_sum.x += gyro_samples[i].x; g_sum.y += gyro_samples[i].y; g_sum.z += gyro_samples[i].z;
    }

    imu->gyro_bias_dps.x = g_sum.x / count;
    imu->gyro_bias_dps.y = g_sum.y / count;
    imu->gyro_bias_dps.z = g_sum.z / count;

    float ax = a_sum.x / count;
    float ay = a_sum.y / count;
    float az = a_sum.z / count;

    float roll0 = atan2f(ay, az);
    float pitch0 = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll0 * 0.5f), sr = sinf(roll0 * 0.5f);
    float cp = cosf(pitch0 * 0.5f), sp = sinf(pitch0 * 0.5f);

    imu->q.w = cr * cp;
    imu->q.x = sr * cp;
    imu->q.y = cr * sp;
    imu->q.z = -sr * sp;

    imu->q_cal_inv = quat_conjugate(imu->q);
    imu->q_segment = (quaternion_t){ 1.0f, 0.0f, 0.0f, 0.0f };
}

void imu_update(imu_state_t* imu, vec3_t accel_g, vec3_t gyro_dps, float dt)
{
    float gx = (gyro_dps.x - imu->gyro_bias_dps.x) * DEG_TO_RAD;
    float gy = (gyro_dps.y - imu->gyro_bias_dps.y) * DEG_TO_RAD;
    float gz = (gyro_dps.z - imu->gyro_bias_dps.z) * DEG_TO_RAD;

    float ax = accel_g.x, ay = accel_g.y, az = accel_g.z;
    float q0 = imu->q.w, q1 = imu->q.x, q2 = imu->q.y, q3 = imu->q.z;
    float norm = sqrtf(ax * ax + ay * ay + az * az);

    if (norm > 0.85f && norm < 1.15f)
    {
        ax /= norm; ay /= norm; az /= norm;

        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        gx += MAHONY_KP * ex;
        gy += MAHONY_KP * ey;
        gz += MAHONY_KP * ez;
    }

    float half_dt = 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * half_dt;
    q1 += (qa * gx + qc * gz - q3 * gy) * half_dt;
    q2 += (qa * gy - qb * gz + q3 * gx) * half_dt;
    q3 += (qa * gz + qb * gy - qc * gx) * half_dt;

    float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    imu->q.w = q0 / q_norm;
    imu->q.x = q1 / q_norm;
    imu->q.y = q2 / q_norm;
    imu->q.z = q3 / q_norm;

    imu->q_segment = quat_multiply(imu->q_cal_inv, imu->q);
}

// --- SYMULATOR KOPNIECIA ---

void simulate_sensor_reading(
    quaternion_t true_bone_q,
    quaternion_t mount_error_q,
    vec3_t angular_vel_world_dps,
    vec3_t linear_accel_world_g,
    vec3_t true_gyro_bias,
    vec3_t* out_accel_g,
    vec3_t* out_gyro_dps)
{
    quaternion_t sensor_world_q = quat_multiply(true_bone_q, mount_error_q);
    quaternion_t world_to_sensor = quat_conjugate(sensor_world_q);

    vec3_t total_acc_world = {
        linear_accel_world_g.x,
        linear_accel_world_g.y,
        linear_accel_world_g.z + 1.0f
    };

    *out_accel_g = quat_rotate_vec(world_to_sensor, total_acc_world);

    vec3_t gyro_local = quat_rotate_vec(world_to_sensor, angular_vel_world_dps);
    out_gyro_dps->x = gyro_local.x + true_gyro_bias.x;
    out_gyro_dps->y = gyro_local.y + true_gyro_bias.y;
    out_gyro_dps->z = gyro_local.z + true_gyro_bias.z;
}

int main(void)
{
    imu_state_t imus[3];

    quaternion_t mount_err[3] = {
        quat_from_axis_angle((vec3_t) { 1, 0, 0 }, 15.0f * DEG_TO_RAD),
        quat_from_axis_angle((vec3_t) { 0, 1, 0 }, -20.0f * DEG_TO_RAD),
        quat_from_axis_angle((vec3_t) { 1, 0, 0 }, 25.0f * DEG_TO_RAD)
    };
    vec3_t fake_bias[3] = {
        {1.2f, -0.8f, 0.5f},
        {-2.1f, 1.5f, -0.3f},
        {0.7f, 0.9f, -1.4f}
    };

    vec3_t cal_acc[3][50], cal_gyro[3][50];
    quaternion_t identity = { 1, 0, 0, 0 };
    vec3_t zero = { 0, 0, 0 };

    for (int s = 0; s < 50; s++) {
        for (int id = 0; id < 3; id++) {
            simulate_sensor_reading(identity, mount_err[id], zero, zero, fake_bias[id],
                &cal_acc[id][s], &cal_gyro[id][s]);
        }
    }
    for (int id = 0; id < 3; id++) {
        imu_calibrate_from_samples(&imus[id], id, cal_acc[id], cal_gyro[id], 50);
    }

    float dt = 0.01f;
    float prev_thigh_ang = 0, prev_shin_ang = 0, prev_foot_ang = 0;

    printf("TIME_S | KNEE(X,Z)     | ANKLE(X,Z)    | TOE(X,Z)      | KAT_UDA | KAT_PISZCZELI\n");
    printf("---------------------------------------------------------------------------------\n");

    for (int step = 0; step <= 200; step++)
    {
        float t = step * dt;

        float kick_envelope = 0.0f;
        if (t >= 0.2f && t <= 1.6f) {
            float phase = (t - 0.2f) / 1.4f * 3.14159265f;
            kick_envelope = sinf(phase) * sinf(phase);
        }

        float thigh_angle = -70.0f * kick_envelope;
        float shin_angle = -85.0f * kick_envelope;
        float foot_angle = -110.0f * kick_envelope;

        float thigh_vel = (thigh_angle - prev_thigh_ang) / dt;
        float shin_vel = (shin_angle - prev_shin_ang) / dt;
        float foot_vel = (foot_angle - prev_foot_ang) / dt;
        prev_thigh_ang = thigh_angle; prev_shin_ang = shin_angle; prev_foot_ang = foot_angle;

        vec3_t kick_g_spike = { 10.0f * kick_envelope, 0.0f, 5.0f * kick_envelope };

        float angles[3] = { thigh_angle, shin_angle, foot_angle };
        float vels[3] = { thigh_vel, shin_vel, foot_vel };

        for (int id = 0; id < 3; id++)
        {
            quaternion_t true_q = quat_from_axis_angle((vec3_t) { 0, 1, 0 }, angles[id] * DEG_TO_RAD);
            vec3_t acc_in, gyro_in;
            simulate_sensor_reading(true_q, mount_err[id], (vec3_t) { 0, vels[id], 0 },
                kick_g_spike, fake_bias[id], & acc_in, & gyro_in);

            imu_update(&imus[id], acc_in, gyro_in, dt);
        }

        vec3_t knee = quat_rotate_vec(imus[0].q_segment, (vec3_t) { 0, 0, -THIGH_LEN });

        vec3_t shin_vec = quat_rotate_vec(imus[1].q_segment, (vec3_t) { 0, 0, -SHIN_LEN });
        vec3_t ankle = { knee.x + shin_vec.x, knee.y + shin_vec.y, knee.z + shin_vec.z };

        vec3_t foot_vec = quat_rotate_vec(imus[2].q_segment, (vec3_t) { FOOT_LEN, 0, 0 });
        vec3_t toe = { ankle.x + foot_vec.x, ankle.y + foot_vec.y, ankle.z + foot_vec.z };

        if (step % 10 == 0)
        {
            float r0, p0, y0, r1, p1, y1;
            quat_to_euler_deg(imus[0].q_segment, &r0, &p0, &y0);
            quat_to_euler_deg(imus[1].q_segment, &r1, &p1, &y1);

            printf("%5.2fs | (%5.2f,%6.2f) | (%5.2f,%6.2f) | (%5.2f,%6.2f) | %6.1f deg | %6.1f deg\n",
                t,
                knee.x, knee.z,
                ankle.x, ankle.z,
                toe.x, toe.z,
                -p0, -p1);
        }
    }

    return 0;
}