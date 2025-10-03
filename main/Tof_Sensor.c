#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "vl53l7cx_api.h"
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

    // --- Initialization ---
    ESP_ERROR_CHECK(i2c_master_init());
    Sensor_Config.platform.address = 0x29; 
    Sensor_Config.platform.port = I2C_MASTER_NUM;
    
    status = vl53l7cx_init(&Sensor_Config);
    if (status) {
        printf("vl53l7cx_init failed, status: %u\n", status);
        return;
    }

    // Set 8x8 resolution
    status = vl53l7cx_set_resolution(&Sensor_Config, VL53L7CX_RESOLUTION_8X8);
    if (status) {
        printf("vl53l7cx_set_resolution failed, status: %u\n", status);
        return;
    }

    // Set the ranging frequency to the maximum for 8x8 mode (15 Hz)
    status = vl53l7cx_set_ranging_frequency_hz(&Sensor_Config, 15);
    if (status) {
        printf("vl53l7cx_set_ranging_frequency_hz failed, status: %u\n", status);
        return;
    }

    // --- Start Ranging ---
    status = vl53l7cx_start_ranging(&Sensor_Config);
    if (status) {
        printf("vl53l7cx_start_ranging failed, status: %u\n", status);
        return;
    }
    // Print some initial newlines to make space for the updating frame
    printf("\n\n\n\n\n\n\n\n\n\n");

    bool first_frame = true;

    // --- Main Loop ---
    while(1) {
        status = vl53l7cx_check_data_ready(&Sensor_Config, &is_ready);
        if (status == 0 && is_ready) {
            
            if (!first_frame) {
                printf("\033[10A"); // Moves cursor up 10 lines
            }
            first_frame = false;
            
            vl53l7cx_get_ranging_data(&Sensor_Config, &results);

            printf("--- VL53L7CX Depth Map ---\n");
            for (int i = 0; i < 64; i++) {
                
                // *** NEW: Logic to choose character based on distance ***
                int distance = results.distance_mm[i];
                char display_char = ' '; // Default to a space

                if (distance > 0 && distance < 250) {
                    display_char = '#'; // Very close
                } else if (distance < 500) {
                    display_char = '&'; // Close
                } else if (distance < 1000) {
                    display_char = '*'; // Medium
                } else if (distance < 1600) {
                    display_char = '.'; // Far
                }
                
                // Print the character twice to make the "pixel" more square
                printf("%c%c ", display_char, display_char);

                if ((i + 1) % 8 == 0) {
                    printf("\n");
                }
            }
            printf("\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}