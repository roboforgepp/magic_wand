#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDR_DEFAULT  0x68
#define MPU6050_I2C_ADDR_ALT      0x69
#define MPU6050_WHO_AM_I_VAL      0x68
#define MPU6500_WHO_AM_I_VAL      0x70
#define MPU_COMPAT_WHO_AM_I_VAL   0x72

#define GRAVITY_STANDARD          9.80665f

typedef enum {
    MPU6050_ACCEL_FS_2G = 0,   /**< ±2g  (16384 LSB/g) */
    MPU6050_ACCEL_FS_4G = 1,   /**< ±4g  (8192 LSB/g) */
    MPU6050_ACCEL_FS_8G = 2,   /**< ±8g  (4096 LSB/g) */
    MPU6050_ACCEL_FS_16G = 3,  /**< ±16g (2048 LSB/g) */
} mpu6050_accel_fs_t;

typedef enum {
    MPU6050_GYRO_FS_250DPS = 0,  /**< ±250 °/s  (131 LSB/(°/s)) */
    MPU6050_GYRO_FS_500DPS = 1,  /**< ±500 °/s  (65.5 LSB/(°/s)) */
    MPU6050_GYRO_FS_1000DPS = 2, /**< ±1000 °/s (32.8 LSB/(°/s)) */
    MPU6050_GYRO_FS_2000DPS = 3, /**< ±2000 °/s (16.4 LSB/(°/s)) */
} mpu6050_gyro_fs_t;

typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6,
} mpu6050_dlpf_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_data_t;

typedef struct {
    float accel_x; /**< Acceleration in m/s^2 */
    float accel_y; /**< Acceleration in m/s^2 */
    float accel_z; /**< Acceleration in m/s^2 */
    float gyro_x;  /**< Angular velocity in deg/s */
    float gyro_y;  /**< Angular velocity in deg/s */
    float gyro_z;  /**< Angular velocity in deg/s */
    float temp_c;  /**< Internal temperature in °C */
} mpu6050_data_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    uint8_t dev_addr;
    uint32_t scl_speed_hz;
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t gyro_fs;
    mpu6050_dlpf_t dlpf;
} mpu6050_config_t;

typedef struct mpu6050_dev_t *mpu6050_handle_t;

/**
 * @brief Initialize MPU6050 sensor on the specified I2C master bus.
 *
 * @param config Pointer to configuration struct.
 * @param[out] out_handle Pointer to store returned device handle.
 * @return ESP_OK on success, or appropriate error code.
 */
esp_err_t mpu6050_create(const mpu6050_config_t *config, mpu6050_handle_t *out_handle);

/**
 * @brief Free resources associated with MPU6050 handle.
 */
esp_err_t mpu6050_delete(mpu6050_handle_t handle);

/**
 * @brief Read 14 raw data bytes (accel, temp, gyro) in a single atomic burst.
 */
esp_err_t mpu6050_read_raw(mpu6050_handle_t handle, mpu6050_raw_data_t *raw_data);

/**
 * @brief Read scaled and bias-compensated sensor data.
 */
esp_err_t mpu6050_read_sensors(mpu6050_handle_t handle, mpu6050_data_t *data);

/**
 * @brief Set zero-rate gyro bias offsets (subtracted during mpu6050_read_sensors).
 */
esp_err_t mpu6050_set_gyro_bias(mpu6050_handle_t handle, float gx, float gy, float gz);

/**
 * @brief Set accelerometer bias offsets.
 */
esp_err_t mpu6050_set_accel_bias(mpu6050_handle_t handle, float ax, float ay, float az);

/**
 * @brief Perform static calibration by averaging N samples while sensor is stationary.
 */
esp_err_t mpu6050_calibrate_gyro(mpu6050_handle_t handle, uint16_t sample_count, float *out_gx, float *out_gy, float *out_gz);

#ifdef __cplusplus
}
#endif
