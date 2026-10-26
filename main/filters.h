#pragma once

/**
 * @file filters.h
 * @brief Digital Signal Processing filters for IMU sensor fusion.
 *
 * Implements:
 *   1. Circular-buffer Moving Average Filter (O(1) time complexity) for noise suppression.
 *   2. Complementary Filter fusing accelerometer gravity vectors and gyroscope rates
 *      for drift-free roll and pitch estimation.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOVING_AVG_DEFAULT_WINDOW 8

/**
 * @brief Circular buffer moving average state structure.
 */
typedef struct {
    float *buffer;      /**< Pointer to user-allocated sample array */
    size_t capacity;    /**< Maximum window size */
    size_t count;       /**< Current number of valid samples */
    size_t head;        /**< Write index in circular ring */
    float sum;          /**< Running sum for O(1) average calculation */
    uint16_t resync_ctr;/**< Counter for periodic exact-sum re-accumulation */
} moving_avg_filter_t;

/**
 * @brief Initialize a moving average filter instance.
 *
 * @param f        Pointer to filter struct
 * @param buffer   Pointer to pre-allocated float buffer of size `capacity`
 * @param capacity Number of samples in the moving average window
 */
void moving_avg_init(moving_avg_filter_t *f, float *buffer, size_t capacity);

/**
 * @brief Insert a new sample into the filter and return the updated average.
 *
 * Time Complexity: O(1)
 *
 * @param f       Pointer to filter
 * @param new_val New sample value
 * @return Computed moving average across window
 */
float moving_avg_update(moving_avg_filter_t *f, float new_val);

/**
 * @brief Complementary filter state structure for attitude estimation.
 */
typedef struct {
    float alpha;        /**< Filter weighting factor (typically 0.95 - 0.98) */
    float roll_deg;     /**< Estimated roll angle around X-axis in degrees */
    float pitch_deg;    /**< Estimated pitch angle around Y-axis in degrees */
    bool initialized;   /**< True once seeded with initial accelerometer tilt */
} complementary_filter_t;

/**
 * @brief Initialize a complementary filter.
 *
 * @param f     Pointer to filter instance
 * @param alpha Weighting factor for gyro integration (e.g. 0.96f)
 */
void complementary_filter_init(complementary_filter_t *f, float alpha);

/**
 * @brief Update attitude estimation with latest sensor readings.
 *
 * @param f  Pointer to filter
 * @param ax Accelerometer X in g
 * @param ay Accelerometer Y in g
 * @param az Accelerometer Z in g
 * @param gx Gyroscope X angular rate in °/s
 * @param gy Gyroscope Y angular rate in °/s
 * @param dt Time delta since previous update in seconds
 */
void complementary_filter_update(complementary_filter_t *f,
                                 float ax, float ay, float az,
                                 float gx, float gy,
                                 float dt);

#ifdef __cplusplus
}
#endif
