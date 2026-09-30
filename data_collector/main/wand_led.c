#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wand_config.h"
#include "wand_led.h"

#if CONFIG_WAND_WS2812_ENABLED
#include "led_strip.h"
#endif

static const char *TAG = "wand_led";

#if CONFIG_WAND_WS2812_ENABLED
static led_strip_handle_t s_led_strip = NULL;
#endif

static volatile wand_state_t s_current_state = WAND_STATE_INIT;

esp_err_t wand_led_init(void)
{
#if CONFIG_WAND_WS2812_ENABLED
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_WAND_WS2812_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags = {
            .with_dma = false,
        }
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WS2812B on GPIO %d: %s", CONFIG_WAND_WS2812_GPIO, esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(s_led_strip);
    ESP_LOGI(TAG, "WS2812B initialized on GPIO %d", CONFIG_WAND_WS2812_GPIO);
    return ESP_OK;
#else
    ESP_LOGI(TAG, "WS2812B disabled by configuration");
    return ESP_OK;
#endif
}

void wand_led_set_state(wand_state_t state)
{
    s_current_state = state;
}

void wand_led_task(void *pvParameters)
{
#if CONFIG_WAND_WS2812_ENABLED
    uint8_t pulse = 0;
    bool pulse_up = true;

    while (1) {
        if (s_led_strip == NULL) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        switch (s_current_state) {
            case WAND_STATE_CALIBRATING:
                // Yellow pulse
                led_strip_set_pixel(s_led_strip, 0, pulse, pulse * 3 / 4, 0);
                led_strip_refresh(s_led_strip);
                if (pulse_up) {
                    pulse += 3;
                    if (pulse >= 45) pulse_up = false;
                } else {
                    if (pulse <= 3) pulse_up = true;
                    else pulse -= 3;
                }
                vTaskDelay(pdMS_TO_TICKS(30));
                break;

            case WAND_STATE_READY:
                // Green steady
                led_strip_set_pixel(s_led_strip, 0, 0, CONFIG_WAND_WS2812_BRIGHTNESS, 0);
                led_strip_refresh(s_led_strip);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case WAND_STATE_STREAMING:
                // Cyan / Blue gentle breathing
                led_strip_set_pixel(s_led_strip, 0, 0, pulse / 3, pulse);
                led_strip_refresh(s_led_strip);
                if (pulse_up) {
                    pulse += 2;
                    if (pulse >= 40) pulse_up = false;
                } else {
                    if (pulse <= 5) pulse_up = true;
                    else pulse -= 2;
                }
                vTaskDelay(pdMS_TO_TICKS(40));
                break;

            case WAND_STATE_ERROR:
                // Red rapid blink
                led_strip_set_pixel(s_led_strip, 0, CONFIG_WAND_WS2812_BRIGHTNESS * 2, 0, 0);
                led_strip_refresh(s_led_strip);
                vTaskDelay(pdMS_TO_TICKS(100));
                led_strip_clear(s_led_strip);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case WAND_STATE_INIT:
            default:
                led_strip_clear(s_led_strip);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
#else
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}
