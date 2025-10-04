#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "vl53l7cx_api.h"
#include "mpu6050.h"
#include "driver/ledc.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"

// I2C settings
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0


// VL53L7CX sensor configuration
static VL53L7CX_Configuration Sensor_Config;


// Function to initialize the I2C bus
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    uint8_t status, is_ready;
    VL53L7CX_ResultsData results;
    
    // *** FIX: Declare the handle for the MPU6050 sensor ***
    mpu6050_handle_t mpu6050_dev = NULL; 

    // --- Initialization ---
    ESP_ERROR_CHECK(i2c_master_init());
    Sensor_Config.platform.address = 0x29; 
    Sensor_Config.platform.port = I2C_MASTER_NUM;
    
    status = vl53l7cx_init(&Sensor_Config);
    if (status) {
        printf("vl53l7cx_init failed, status: %u\n", status);
        return;
    }

    status = vl53l7cx_set_resolution(&Sensor_Config, VL53L7CX_RESOLUTION_8X8);
    if (status) {
        printf("vl53l7cx_set_resolution failed, status: %u\n", status);
        return;
    }

    status = vl53l7cx_set_ranging_frequency_hz(&Sensor_Config, 15);
    if (status) {
        printf("vl53l7cx_set_ranging_frequency_hz failed, status: %u\n", status);
        return;
    }

    // MPU6050 Sensor Initialization
    mpu6050_dev = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu6050_dev));
    ESP_ERROR_CHECK(mpu6050_config(mpu6050_dev, ACCE_FS_2G, GYRO_FS_250DPS));
    printf("MPU6050 initialized successfully!\n");

    // --- Start Ranging ---
    status = vl53l7cx_start_ranging(&Sensor_Config);
    if (status) {
        printf("vl53l7cx_start_ranging failed, status: %u\n", status);
        return;
    }
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n"); // Make space

    bool first_frame = true;

    // --- Main Loop ---
    while(1) {
        status = vl53l7cx_check_data_ready(&Sensor_Config, &is_ready);
        if (status == 0 && is_ready) {
            
            if (!first_frame) {
                // *** FIX: Adjust cursor move for the combined display size ***
                printf("\033[15A"); 
            }
            first_frame = false;
            
            // --- Get VL53L7CX Data ---
            vl53l7cx_get_ranging_data(&Sensor_Config, &results);
            
            // *** FIX: Declare structs AND get the MPU6050 data ***
            mpu6050_acce_value_t acce;
            mpu6050_gyro_value_t gyro;
            ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050_dev, &acce));
            ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050_dev, &gyro));

            // --- Print Combined Display ---
            for (int i = 0; i < 64; i++) {
                printf("%4d ", results.distance_mm[i]);
                if ((i + 1) % 8 == 0) {
                    printf("\n");
                }
            }
            
            printf("--------------------------------------------------\n");
            printf("MPU6050 Accel: X=%6.2f | Y=%6.2f | Z=%6.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
            printf("MPU6050 Gyro:  X=%6.2f | Y=%6.2f | Z=%6.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
            printf("\n");
        }
        
        
        
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}