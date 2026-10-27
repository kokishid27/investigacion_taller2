idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mpu_6050_driver esp_timer driver esp_driver_gpio)