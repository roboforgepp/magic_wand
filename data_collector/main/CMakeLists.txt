idf_component_register(SRCS "main.c" "wand_telemetry.c" "wand_led.c"
                       INCLUDE_DIRS "."
                       REQUIRES mpu6050 led_strip esp_driver_i2c esp_timer)
