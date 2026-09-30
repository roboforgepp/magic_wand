#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "wand_config.h"
#include "wand_telemetry.h"

static const char *TAG = "wand_telem";

static QueueHandle_t s_imu_queue = NULL;
static telemetry_stats_t s_stats = {0};

esp_err_t wand_telemetry_init(void)
{
    if (s_imu_queue != NULL) {
        return ESP_OK;
    }

    s_imu_queue = xQueueCreate(CONFIG_WAND_QUEUE_SIZE, sizeof(imu_sample_t));
    if (s_imu_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU sample queue");
        return ESP_ERR_NO_MEM;
    }

    s_stats.samples_sent = 0;
    s_stats.samples_dropped = 0;

    ESP_LOGI(TAG, "Telemetry queue initialized (Capacity: %d frames)", CONFIG_WAND_QUEUE_SIZE);
    return ESP_OK;
}

bool wand_telemetry_send_sample(const imu_sample_t *sample)
{
    if (s_imu_queue == NULL || sample == NULL) {
        return false;
    }

    // Non-blocking send: never delay real-time sensor task
    BaseType_t ret = xQueueSend(s_imu_queue, sample, 0);
    if (ret == pdTRUE) {
        s_stats.samples_sent++;
        return true;
    } else {
        s_stats.samples_dropped++;
        return false;
    }
}

void wand_telemetry_get_stats(telemetry_stats_t *out_stats)
{
    if (out_stats) {
        *out_stats = s_stats;
    }
}

QueueHandle_t wand_telemetry_get_queue(void)
{
    return s_imu_queue;
}

void wand_telemetry_task(void *pvParameters)
{
    imu_sample_t sample;
    ESP_LOGI(TAG, "Telemetry task started on Core %d", xPortGetCoreID());

    while (1) {
        if (xQueueReceive(s_imu_queue, &sample, portMAX_DELAY) == pdTRUE) {
#if CONFIG_WAND_TELEMETRY_INCLUDE_TIMESTAMP
            printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                   (unsigned long)sample.timestamp_ms,
                   sample.acc_x, sample.acc_y, sample.acc_z,
                   sample.gyr_x, sample.gyr_y, sample.gyr_z);
#else
            printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                   sample.acc_x, sample.acc_y, sample.acc_z,
                   sample.gyr_x, sample.gyr_y, sample.gyr_z);
#endif
            fflush(stdout);
        }
    }
}
