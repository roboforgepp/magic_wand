#pragma once

#include "esp_err.h"
#include "wand_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize WS2812B LED driver (RMT backend).
 */
esp_err_t wand_led_init(void);

/**
 * @brief Set the current status state for LED pattern display.
 */
void wand_led_set_state(wand_state_t state);

/**
 * @brief FreeRTOS task handling LED animations.
 */
void wand_led_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
