#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "mpu6050.h"
#include "wand_config.h"
#include "wand_types.h"
#include "wand_telemetry.h"
#include "wand_led.h"

static const char *TAG = "magic_wand";

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static mpu6050_handle_t s_mpu6050 = NULL;

static void sensor_task(void *pvParameters)
{
    mpu6050_handle_t mpu = (mpu6050_handle_t)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONFIG_WAND_SAMPLE_RATE_HZ);

    ESP_LOGI(TAG, "Sensor acquisition task started on Core %d (Rate: %d Hz, Period: %" PRIu32 " ms)",
             xPortGetCoreID(), CONFIG_WAND_SAMPLE_RATE_HZ, (uint32_t)(1000 / CONFIG_WAND_SAMPLE_RATE_HZ));

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        mpu6050_data_t data;
        esp_err_t err = mpu6050_read_sensors(mpu, &data);
        if (err == ESP_OK) {
            imu_sample_t sample = {
                .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
                .acc_x = data.accel_x,
                .acc_y = data.accel_y,
                .acc_z = data.accel_z,
                .gyr_x = data.gyro_x,
                .gyr_y = data.gyro_y,
                .gyr_z = data.gyro_z,
                .temp_c = data.temp_c,
            };
            wand_telemetry_send_sample(&sample);
        } else {
            ESP_LOGE(TAG, "I2C read error: %s", esp_err_to_name(err));
            wand_led_set_state(WAND_STATE_ERROR);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "   Magic Wand Data Collector for XIAO ESP32S3  ");
    ESP_LOGI(TAG, "===============================================");

    // 1. Initialize WS2812B LED & start background feedback task
    ESP_ERROR_CHECK(wand_led_init());
    xTaskCreatePinnedToCore(
        wand_led_task,
        "led_task",
        LED_TASK_STACK_SIZE,
        NULL,
        LED_TASK_PRIORITY,
        NULL,
        LED_TASK_CORE_ID
    );
    wand_led_set_state(WAND_STATE_INIT);

    // 2. Initialize Telemetry Queue & Start Telemetry Task
    ESP_ERROR_CHECK(wand_telemetry_init());
    xTaskCreatePinnedToCore(
        wand_telemetry_task,
        "telemetry_task",
        TELEMETRY_TASK_STACK_SIZE,
        NULL,
        TELEMETRY_TASK_PRIORITY,
        NULL,
        TELEMETRY_TASK_CORE_ID
    );

    // 3. Initialize I2C Master Bus (driver/i2c_master.h)
    ESP_LOGI(TAG, "Initializing I2C Master (SDA: GPIO %d, SCL: GPIO %d, Clock: %d Hz)...",
             CONFIG_WAND_I2C_SDA_GPIO, CONFIG_WAND_I2C_SCL_GPIO, CONFIG_WAND_I2C_FREQ_HZ);

    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_WAND_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_WAND_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&i2c_bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        wand_led_set_state(WAND_STATE_ERROR);
        return;
    }

    // 4. Initialize MPU6050
    mpu6050_config_t mpu_cfg = {
        .bus_handle = s_i2c_bus,
        .dev_addr = CONFIG_WAND_MPU6050_I2C_ADDR,
        .scl_speed_hz = CONFIG_WAND_I2C_FREQ_HZ,
        .accel_fs = MPU6050_ACCEL_FS_8G,
        .gyro_fs = MPU6050_GYRO_FS_2000DPS,
        .dlpf = MPU6050_DLPF_44HZ,
    };

    err = mpu6050_create(&mpu_cfg, &s_mpu6050);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050 sensor!");
        wand_led_set_state(WAND_STATE_ERROR);
        return;
    }

    // 5. Stationary Gyro Bias Calibration
    wand_led_set_state(WAND_STATE_CALIBRATING);
    ESP_LOGI(TAG, "Keep the magic wand motionless for calibration (%d samples)...", CONFIG_WAND_CALIBRATION_SAMPLES);
    err = mpu6050_calibrate_gyro(s_mpu6050, CONFIG_WAND_CALIBRATION_SAMPLES, NULL, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Gyro calibration had issues, proceeding with default bias.");
    }

    wand_led_set_state(WAND_STATE_READY);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 6. Launch high-priority periodic sensor acquisition task on Core 1
    wand_led_set_state(WAND_STATE_STREAMING);
    ESP_LOGI(TAG, "Starting IMU streaming at %d Hz...", CONFIG_WAND_SAMPLE_RATE_HZ);

    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        SENSOR_TASK_STACK_SIZE,
        (void *)s_mpu6050,
        SENSOR_TASK_PRIORITY,
        NULL,
        SENSOR_TASK_CORE_ID
    );

    // 7. Supervisory loop (Core 0): Reports memory and telemetry stats every 10 seconds
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        telemetry_stats_t stats;
        wand_telemetry_get_stats(&stats);
        ESP_LOGD(TAG, "Heartbeat - Free Heap: %" PRIu32 " B | Sent: %" PRIu32 " | Dropped: %" PRIu32,
                 esp_get_free_heap_size(), stats.samples_sent, stats.samples_dropped);
    }
}
