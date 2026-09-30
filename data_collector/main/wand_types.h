#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WAND_STATE_INIT = 0,
    WAND_STATE_CALIBRATING,
    WAND_STATE_READY,
    WAND_STATE_STREAMING,
    WAND_STATE_ERROR,
} wand_state_t;

typedef struct {
    uint32_t timestamp_ms;  /**< Monotonic timestamp in milliseconds */
    float acc_x;            /**< Accelerometer X in m/s^2 */
    float acc_y;            /**< Accelerometer Y in m/s^2 */
    float acc_z;            /**< Accelerometer Z in m/s^2 */
    float gyr_x;            /**< Gyroscope X in deg/s (dps) */
    float gyr_y;            /**< Gyroscope Y in deg/s (dps) */
    float gyr_z;            /**< Gyroscope Z in deg/s (dps) */
    float temp_c;           /**< Sensor temperature in °C */
} imu_sample_t;

typedef struct {
    uint32_t samples_sent;
    uint32_t samples_dropped;
} telemetry_stats_t;

#ifdef __cplusplus
}
#endif
