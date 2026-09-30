#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

static const char *TAG = "mpu6050";

/* Register Map */
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_WHO_AM_I         0x75

struct mpu6050_dev_t {
    i2c_master_dev_handle_t i2c_dev;
    float accel_sensitivity;
    float gyro_sensitivity;
    float gyro_bias[3];
    float accel_bias[3];
};

static esp_err_t write_register(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buffer[2] = {reg, val};
    return i2c_master_transmit(dev, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
}

static esp_err_t read_registers(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

esp_err_t mpu6050_create(const mpu6050_config_t *config, mpu6050_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL || config->bus_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct mpu6050_dev_t *dev = calloc(1, sizeof(struct mpu6050_dev_t));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->dev_addr ? config->dev_addr : MPU6050_I2C_ADDR_DEFAULT,
        .scl_speed_hz = config->scl_speed_hz ? config->scl_speed_hz : 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(config->bus_handle, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6050 to I2C master bus: %s", esp_err_to_name(ret));
        free(dev);
        return ret;
    }

    // Verify WHO_AM_I
    uint8_t who_am_i = 0;
    ret = read_registers(dev->i2c_dev, MPU6050_RA_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return ret;
    }

    // Check for MPU6050 (0x68) or compatible sensors (0x70, 0x72)
    uint8_t id = who_am_i & 0x7E;
    if (id == MPU6050_WHO_AM_I_VAL || who_am_i == MPU_COMPAT_WHO_AM_I_VAL || who_am_i == MPU6500_WHO_AM_I_VAL) {
        ESP_LOGI(TAG, "IMU detected with WHO_AM_I = 0x%02X", who_am_i);
    } else {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I value: 0x%02X (expected 0x%02X, 0x%02X, or 0x%02X)",
                 who_am_i, MPU6050_WHO_AM_I_VAL, MPU6500_WHO_AM_I_VAL, MPU_COMPAT_WHO_AM_I_VAL);
    }

    // Reset device
    write_register(dev->i2c_dev, MPU6050_RA_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up device and select PLL with X axis gyroscope reference
    ret = write_register(dev->i2c_dev, MPU6050_RA_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set digital low pass filter
    write_register(dev->i2c_dev, MPU6050_RA_CONFIG, config->dlpf & 0x07);

    // Set sample rate divider (0 = 1kHz gyro rate)
    write_register(dev->i2c_dev, MPU6050_RA_SMPLRT_DIV, 0x00);

    // Configure Gyro full-scale range
    write_register(dev->i2c_dev, MPU6050_RA_GYRO_CONFIG, (config->gyro_fs & 0x03) << 3);
    switch (config->gyro_fs) {
        case MPU6050_GYRO_FS_250DPS:  dev->gyro_sensitivity = 131.0f; break;
        case MPU6050_GYRO_FS_500DPS:  dev->gyro_sensitivity = 65.5f; break;
        case MPU6050_GYRO_FS_1000DPS: dev->gyro_sensitivity = 32.8f; break;
        case MPU6050_GYRO_FS_2000DPS:
        default:                      dev->gyro_sensitivity = 16.4f; break;
    }

    // Configure Accel full-scale range
    write_register(dev->i2c_dev, MPU6050_RA_ACCEL_CONFIG, (config->accel_fs & 0x03) << 3);
    switch (config->accel_fs) {
        case MPU6050_ACCEL_FS_2G:  dev->accel_sensitivity = 16384.0f; break;
        case MPU6050_ACCEL_FS_4G:  dev->accel_sensitivity = 8192.0f; break;
        case MPU6050_ACCEL_FS_16G: dev->accel_sensitivity = 2048.0f; break;
        case MPU6050_ACCEL_FS_8G:
        default:                   dev->accel_sensitivity = 4096.0f; break;
    }

    *out_handle = dev;
    ESP_LOGI(TAG, "MPU6050 initialized successfully (Accel FS: %d, Gyro FS: %d)",
             config->accel_fs, config->gyro_fs);
    return ESP_OK;
}

esp_err_t mpu6050_delete(mpu6050_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = i2c_master_bus_rm_device(handle->i2c_dev);
    free(handle);
    return ret;
}

esp_err_t mpu6050_read_raw(mpu6050_handle_t handle, mpu6050_raw_data_t *raw_data)
{
    if (handle == NULL || raw_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[14];
    esp_err_t ret = read_registers(handle->i2c_dev, MPU6050_RA_ACCEL_XOUT_H, buffer, sizeof(buffer));
    if (ret != ESP_OK) {
        return ret;
    }

    raw_data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    raw_data->temp    = (int16_t)((buffer[6] << 8) | buffer[7]);
    raw_data->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw_data->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw_data->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}

esp_err_t mpu6050_read_sensors(mpu6050_handle_t handle, mpu6050_data_t *data)
{
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu6050_raw_data_t raw;
    esp_err_t ret = mpu6050_read_raw(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Accelerometer scaled to m/s^2 (raw / sensitivity * 9.80665)
    data->accel_x = (((float)raw.accel_x / handle->accel_sensitivity) * GRAVITY_STANDARD) - handle->accel_bias[0];
    data->accel_y = (((float)raw.accel_y / handle->accel_sensitivity) * GRAVITY_STANDARD) - handle->accel_bias[1];
    data->accel_z = (((float)raw.accel_z / handle->accel_sensitivity) * GRAVITY_STANDARD) - handle->accel_bias[2];

    // Gyroscope scaled to deg/s (dps) minus stationary bias
    data->gyro_x = ((float)raw.gyro_x / handle->gyro_sensitivity) - handle->gyro_bias[0];
    data->gyro_y = ((float)raw.gyro_y / handle->gyro_sensitivity) - handle->gyro_bias[1];
    data->gyro_z = ((float)raw.gyro_z / handle->gyro_sensitivity) - handle->gyro_bias[2];

    // Temperature in °C
    data->temp_c = ((float)raw.temp / 340.0f) + 36.53f;

    return ESP_OK;
}

esp_err_t mpu6050_set_gyro_bias(mpu6050_handle_t handle, float gx, float gy, float gz)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->gyro_bias[0] = gx;
    handle->gyro_bias[1] = gy;
    handle->gyro_bias[2] = gz;
    return ESP_OK;
}

esp_err_t mpu6050_set_accel_bias(mpu6050_handle_t handle, float ax, float ay, float az)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->accel_bias[0] = ax;
    handle->accel_bias[1] = ay;
    handle->accel_bias[2] = az;
    return ESP_OK;
}

esp_err_t mpu6050_calibrate_gyro(mpu6050_handle_t handle, uint16_t sample_count, float *out_gx, float *out_gy, float *out_gz)
{
    if (handle == NULL || sample_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting gyro calibration (%u samples, keep wand stationary)...", sample_count);

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;

    // Temporarily clear bias
    handle->gyro_bias[0] = 0.0f;
    handle->gyro_bias[1] = 0.0f;
    handle->gyro_bias[2] = 0.0f;

    uint16_t valid_samples = 0;
    for (uint16_t i = 0; i < sample_count; i++) {
        mpu6050_data_t sample;
        if (mpu6050_read_sensors(handle, &sample) == ESP_OK) {
            sum_gx += sample.gyro_x;
            sum_gy += sample.gyro_y;
            sum_gz += sample.gyro_z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid_samples == 0) {
        ESP_LOGE(TAG, "Gyro calibration failed: no valid samples");
        return ESP_FAIL;
    }

    float avg_gx = sum_gx / (float)valid_samples;
    float avg_gy = sum_gy / (float)valid_samples;
    float avg_gz = sum_gz / (float)valid_samples;

    handle->gyro_bias[0] = avg_gx;
    handle->gyro_bias[1] = avg_gy;
    handle->gyro_bias[2] = avg_gz;

    if (out_gx) *out_gx = avg_gx;
    if (out_gy) *out_gy = avg_gy;
    if (out_gz) *out_gz = avg_gz;

    ESP_LOGI(TAG, "Gyro calibration completed: Bias = (%.2f, %.2f, %.2f) dps",
             avg_gx, avg_gy, avg_gz);
    return ESP_OK;
}
