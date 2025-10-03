idf_component_register(SRCS "Tof_Sensor.c"
                    INCLUDE_DIRS "."
                    REQUIRES vl53l7cx
                    mpu6050 
                    spiffs 
                    esp_driver_gpio 
                    esp_driver_ledc)
