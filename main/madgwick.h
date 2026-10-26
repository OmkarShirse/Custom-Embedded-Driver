#pragma once

/**
 * @file madgwick.h
 * @brief Madgwick AHRS (Attitude and Heading Reference System) quaternion filter.
 *
 * Implements Sebastian Madgwick's gradient-descent orientation filter for 6-DOF
 * (Accelerometer + Gyroscope) inertial measurement units. Computes a singularity-free
 * unit quaternion [q0, q1, q2, q3] and extracts Euler angles (roll, pitch, yaw).
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MADGWICK_DEFAULT_BETA 0.05f     /* Default algorithm gain */

/**
 * @brief Madgwick filter state structure.
 */
typedef struct {
    float q0;   /**< Quaternion scalar component (w) */
    float q1;   /**< Quaternion vector X component (x) */
    float q2;   /**< Quaternion vector Y component (y) */
    float q3;   /**< Quaternion vector Z component (z) */
    float beta; /**< Filter gain representing gyroscope measurement error */
} madgwick_filter_t;

/**
 * @brief Initialize the Madgwick filter instance to identity quaternion [1, 0, 0, 0].
 *
 * @param filter Pointer to filter struct
 * @param beta   Filter gain (typically 0.03 - 0.1 for MEMS IMUs)
 */
void madgwick_init(madgwick_filter_t *filter, float beta);

/**
 * @brief Initialize the Madgwick filter seeded from the initial accelerometer gravity vector.
 *
 * Accelerates convergence from 10-15s down to 0s upon boot.
 *
 * @param filter Pointer to filter struct
 * @param beta   Filter gain
 * @param ax     Initial accelerometer X
 * @param ay     Initial accelerometer Y
 * @param az     Initial accelerometer Z
 */
void madgwick_init_from_accel(madgwick_filter_t *filter, float beta, float ax, float ay, float az);

/**
 * @brief Set the algorithm gain parameter beta.
 *
 * @param filter Pointer to filter struct
 * @param beta   New beta value in rad/s
 */
void madgwick_set_beta(madgwick_filter_t *filter, float beta);

/**
 * @brief 6-DOF Madgwick orientation filter update step.
 *
 * Fuses 3-axis accelerometer and 3-axis gyroscope readings via single-step
 * gradient descent on the orientation error surface.
 *
 * @param filter Pointer to filter struct
 * @param gx_rad Angular velocity X in radians per second (rad/s)
 * @param gy_rad Angular velocity Y in radians per second (rad/s)
 * @param gz_rad Angular velocity Z in radians per second (rad/s)
 * @param ax     Linear acceleration X (in any consistent unit, e.g. g or m/s²)
 * @param ay     Linear acceleration Y (in any consistent unit, e.g. g or m/s²)
 * @param az     Linear acceleration Z (in any consistent unit, e.g. g or m/s²)
 * @param dt     Sampling period / time delta since previous update in seconds
 */
void madgwick_update_6dof(madgwick_filter_t *filter,
                          float gx_rad, float gy_rad, float gz_rad,
                          float ax, float ay, float az,
                          float dt);

/**
 * @brief Extract estimated Euler angles in degrees from the orientation quaternion.
 *
 * @param filter Pointer to filter struct
 * @param roll   Output roll angle (-180° to +180°) around X axis
 * @param pitch  Output pitch angle (-90° to +90°) around Y axis
 * @param yaw    Output yaw angle (-180° to +180°) around Z axis
 */
void madgwick_get_euler_deg(const madgwick_filter_t *filter,
                            float *roll,
                            float *pitch,
                            float *yaw);

/**
 * @brief Retrieve current unit quaternion components.
 *
 * @param filter Pointer to filter struct
 * @param q0     Output scalar (w)
 * @param q1     Output vector X (x)
 * @param q2     Output vector Y (y)
 * @param q3     Output vector Z (z)
 */
void madgwick_get_quaternion(const madgwick_filter_t *filter,
                             float *q0,
                             float *q1,
                             float *q2,
                             float *q3);

#ifdef __cplusplus
}
#endif
