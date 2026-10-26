#include "filters.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / M_PI)

void moving_avg_init(moving_avg_filter_t *f, float *buffer, size_t capacity)
{
    if (f == NULL || buffer == NULL || capacity == 0) return;

    f->buffer   = buffer;
    f->capacity = capacity;
    f->count    = 0;
    f->head     = 0;
    f->sum      = 0.0f;

    for (size_t i = 0; i < capacity; i++) {
        f->buffer[i] = 0.0f;
    }
}

float moving_avg_update(moving_avg_filter_t *f, float new_val)
{
    if (f == NULL || f->buffer == NULL || f->capacity == 0) return new_val;

    if (f->count < f->capacity) {
        /* Buffer is still filling up */
        f->buffer[f->head] = new_val;
        f->sum += new_val;
        f->count++;
        f->head = (f->head + 1) % f->capacity;
        return f->sum / (float)f->count;
    }

    /* Buffer is full: subtract oldest sample, insert new sample (O(1)) */
    f->sum -= f->buffer[f->head];
    f->buffer[f->head] = new_val;
    f->sum += new_val;
    f->head = (f->head + 1) % f->capacity;

    /* Periodic re-accumulation to prevent cumulative single-precision float drift */
    static uint16_t resync_ctr = 0;
    if ((++resync_ctr & 0x00FF) == 0) {
        float exact_sum = 0.0f;
        for (size_t i = 0; i < f->capacity; i++) {
            exact_sum += f->buffer[i];
        }
        f->sum = exact_sum;
    }

    return f->sum / (float)f->capacity;
}

void complementary_filter_init(complementary_filter_t *f, float alpha)
{
    if (f == NULL) return;

    f->alpha       = alpha;
    f->roll_deg    = 0.0f;
    f->pitch_deg   = 0.0f;
    f->initialized = false;
}

void complementary_filter_update(complementary_filter_t *f,
                                 float ax, float ay, float az,
                                 float gx, float gy,
                                 float dt)
{
    if (f == NULL || dt <= 0.0f) return;

    /* Check for near-zero acceleration norm (free-fall / zero-g shock) */
    float accel_sq = ax * ax + ay * ay + az * az;
    if (accel_sq < 0.01f) {
        /* In free fall, trust gyroscope integration only */
        f->roll_deg  += gx * dt;
        f->pitch_deg += gy * dt;
        return;
    }

    /* Compute tilt angles from accelerometer gravity vector */
    /* Roll: rotation around X axis (Y vs Z) */
    float accel_roll = atan2f(ay, az) * RAD_TO_DEG;

    /* Pitch: rotation around Y axis (X vs resultant of Y and Z) */
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    if (!f->initialized) {
        /* Seed the initial angle directly with accelerometer to avoid convergence lag */
        f->roll_deg    = accel_roll;
        f->pitch_deg   = accel_pitch;
        f->initialized = true;
        return;
    }

    /* Phase-unwrapped angular difference to eliminate wrap-around jump across ±180° */
    float diff_roll = accel_roll - f->roll_deg;
    while (diff_roll > 180.0f)  diff_roll -= 360.0f;
    while (diff_roll < -180.0f) diff_roll += 360.0f;

    float diff_pitch = accel_pitch - f->pitch_deg;
    while (diff_pitch > 180.0f)  diff_pitch -= 360.0f;
    while (diff_pitch < -180.0f) diff_pitch += 360.0f;

    /* Complementary filter update with angular difference */
    f->roll_deg  += (gx * dt) + (1.0f - f->alpha) * diff_roll;
    f->pitch_deg += (gy * dt) + (1.0f - f->alpha) * diff_pitch;

    /* Keep roll bounded in [-180, +180] */
    while (f->roll_deg > 180.0f)  f->roll_deg -= 360.0f;
    while (f->roll_deg < -180.0f) f->roll_deg += 360.0f;
}
