set(srcs "main.cpp")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2c esp_driver_gpio nvs_flash esp_http_server esp_wifi
                    PRIV_REQUIRES ESP32_VL53L0X)
