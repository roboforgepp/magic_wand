#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "wand_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the telemetry queue and statistics.
 */
esp_err_t wand_telemetry_init(void);

/**
 * @brief Enqueue a sample from the sensor acquisition task (non-blocking).
 *
 * @param sample Pointer to sample data.
 * @return true if enqueued successfully, false if queue full (sample dropped).
 */
bool wand_telemetry_send_sample(const imu_sample_t *sample);

/**
 * @brief Get queue statistics (sent count, dropped count).
 */
void wand_telemetry_get_stats(telemetry_stats_t *out_stats);

/**
 * @brief FreeRTOS task function for serial streaming.
 */
void wand_telemetry_task(void *pvParameters);

/**
 * @brief Get handle to telemetry queue.
 */
QueueHandle_t wand_telemetry_get_queue(void);

#ifdef __cplusplus
}
#endif
