#include "madgwick.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / M_PI)

static inline float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void madgwick_init(madgwick_filter_t *filter, float beta)
{
    if (filter == NULL) return;

    /* Initialize to identity quaternion [1, 0, 0, 0] */
    filter->q0   = 1.0f;
    filter->q1   = 0.0f;
    filter->q2   = 0.0f;
    filter->q3   = 0.0f;
    filter->beta = (beta > 0.0f) ? beta : MADGWICK_DEFAULT_BETA;
}

void madgwick_init_from_accel(madgwick_filter_t *filter, float beta, float ax, float ay, float az)
{
    if (filter == NULL) return;
    madgwick_init(filter, beta);

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.01f) return;

    ax /= norm;
    ay /= norm;
    az /= norm;

    /* Calculate tilt orientation from gravity vector:
     * roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2 + az^2)) */
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    filter->q0 = cr * cp;
    filter->q1 = sr * cp;
    filter->q2 = cr * sp;
    filter->q3 = -sr * sp;

    float q_norm = sqrtf(filter->q0 * filter->q0 + filter->q1 * filter->q1 +
                         filter->q2 * filter->q2 + filter->q3 * filter->q3);
    if (q_norm > 0.0001f) {
        filter->q0 /= q_norm;
        filter->q1 /= q_norm;
        filter->q2 /= q_norm;
        filter->q3 /= q_norm;
    }
}

void madgwick_set_beta(madgwick_filter_t *filter, float beta)
{
    if (filter != NULL && beta >= 0.0f) {
        filter->beta = beta;
    }
}

void madgwick_update_6dof(madgwick_filter_t *filter,
                          float gx_rad, float gy_rad, float gz_rad,
                          float ax, float ay, float az,
                          float dt)
{
    if (filter == NULL || dt <= 0.0f) return;

    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    /* Rate of change of quaternion from gyroscope */
    float qDot1 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
    float qDot2 = 0.5f * ( q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
    float qDot3 = 0.5f * ( q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
    float qDot4 = 0.5f * ( q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

    /* Compute feedback only if accelerometer measurement valid (avoids NaN in normalisation) */
    float accel_sq = ax * ax + ay * ay + az * az;
    if (accel_sq > 0.00001f) {
        /* Normalise accelerometer measurement */
        float recipNorm = inv_sqrt(accel_sq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        /* Gradient descent objective function gradient: J^T * f */
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        /* Normalise step magnitude */
        float s_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (s_sq > 0.000001f) {
            recipNorm = inv_sqrt(s_sq);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            /* Apply gradient descent feedback to quaternion derivative */
            qDot1 -= filter->beta * s0;
            qDot2 -= filter->beta * s1;
            qDot3 -= filter->beta * s2;
            qDot4 -= filter->beta * s3;
        }
    }

    /* Integrate rate of change to calculate quaternion */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* Normalise quaternion */
    float q_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (q_sq > 0.000001f) {
        float recipNorm = inv_sqrt(q_sq);
        filter->q0 = q0 * recipNorm;
        filter->q1 = q1 * recipNorm;
        filter->q2 = q2 * recipNorm;
        filter->q3 = q3 * recipNorm;
    }
}

void madgwick_get_euler_deg(const madgwick_filter_t *filter,
                            float *roll,
                            float *pitch,
                            float *yaw)
{
    if (filter == NULL) return;

    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    /* Pitch (y-axis rotation): test for gimbal lock at pitch = ±90° */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 0.9999f) {
        if (pitch != NULL) *pitch = copysignf(90.0f, sinp);
        if (roll != NULL)  *roll  = 0.0f;
        if (yaw != NULL)   *yaw   = -2.0f * atan2f(q1, q0) * RAD_TO_DEG;
        return;
    }

    /* Pitch normal domain */
    if (pitch != NULL) {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }

    /* Roll (x-axis rotation) */
    if (roll != NULL) {
        *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    }

    /* Yaw (z-axis rotation) */
    if (yaw != NULL) {
        *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
    }
}

void madgwick_get_quaternion(const madgwick_filter_t *filter,
                             float *q0,
                             float *q1,
                             float *q2,
                             float *q3)
{
    if (filter == NULL) return;
    if (q0 != NULL) *q0 = filter->q0;
    if (q1 != NULL) *q1 = filter->q1;
    if (q2 != NULL) *q2 = filter->q2;
    if (q3 != NULL) *q3 = filter->q3;
}
