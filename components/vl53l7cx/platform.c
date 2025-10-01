/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "include/platform.h"
#include "FreeRTOSConfig.h"
#include "driver/i2c.h"

#define ACK_CHECK_EN                    0x1
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1

uint8_t _i2c_buffer[32769];

int _i2c_write(VL53L7CX_Platform *p_platform, uint8_t *buf, uint32_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (p_platform->address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, buf, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(p_platform->port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return 1;
    }
    return 0;
}

int _i2c_read(VL53L7CX_Platform *p_platform, uint8_t *buf, uint32_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (p_platform->address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(p_platform->port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return 1;
    }
    return 0;
}

uint8_t RdByte(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {

    int32_t status_int;

    _i2c_buffer[0] = RegisterAdress >> 8;
    _i2c_buffer[1] = RegisterAdress & 0xFF;

    status_int = _i2c_write(p_platform, _i2c_buffer, 2);
    if (status_int) {

        return 1;
    }
    return _i2c_read(p_platform, p_value, 1);

}

uint8_t WrByte(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {

    _i2c_buffer[0] = RegisterAdress >> 8;
    _i2c_buffer[1] = RegisterAdress & 0xFF;
    _i2c_buffer[2] = value;

    return _i2c_write(p_platform, _i2c_buffer, 3);
}

uint8_t WrMulti(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    
    #define MAX_CHUNK_SIZE 256
    uint8_t buffer[MAX_CHUNK_SIZE + 2]; // Temp buffer for address + data chunk

    uint32_t bytes_left = size;
    uint32_t bytes_to_write = 0;
    uint32_t current_addr = RegisterAdress;

    while (bytes_left > 0) {
        // Determine the size of the next chunk to send
        bytes_to_write = (bytes_left > MAX_CHUNK_SIZE) ? MAX_CHUNK_SIZE : bytes_left;

        // Prepare the buffer: 2 bytes for the starting register address, then the data chunk
        buffer[0] = current_addr >> 8;
        buffer[1] = current_addr & 0xFF;
        memcpy(&buffer[2], p_values, bytes_to_write);
        
        // Use the _i2c_write helper to send this smaller, manageable transaction
        if (_i2c_write(p_platform, buffer, bytes_to_write + 2) != 0) {
            // The _i2c_write function will print the specific trace error
            return 1;
        }

        // Move pointers to the next chunk
        p_values += bytes_to_write;
        bytes_left -= bytes_to_write;
        current_addr += bytes_to_write;
    }
    return 0;
}

uint8_t RdMulti(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    int32_t status_int;

    _i2c_buffer[0] = RegisterAdress >> 8;
    _i2c_buffer[1] = RegisterAdress & 0xFF;

    status_int = _i2c_write(p_platform, _i2c_buffer, 2);
    if (status_int != 0) {
        return 1;

    }
    status_int = _i2c_read(p_platform, p_values, size);
    if (status_int != 0) {
        return 1;
    }
    return 0;
}

uint8_t Reset_Sensor(VL53L7CX_Platform *p_platform) {
    uint8_t status = 0;

    /* (Optional) Need to be implemented by customer. This function returns 0 if OK */

    /* Set pin LPN to LOW */
    /* Set pin AVDD to LOW */
    /* Set pin VDDIO  to LOW */
    WaitMs(p_platform, 100);

    /* Set pin LPN of to HIGH */
    /* Set pin AVDD of to HIGH */
    /* Set pin VDDIO of  to HIGH */
    WaitMs(p_platform, 100);

    return status;
}

void SwapBuffer(uint8_t *buffer, uint16_t size) {
    uint32_t i, tmp;

    /* Example of possible implementation using <string.h> */
    for (i = 0; i < size; i = i + 4) {
        tmp = (
                      buffer[i] << 24)
              | (buffer[i + 1] << 16)
              | (buffer[i + 2] << 8)
              | (buffer[i + 3]);

        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t WaitMs(VL53L7CX_Platform *p_platform, uint32_t TimeMs) {

    vTaskDelay(TimeMs / portTICK_PERIOD_MS);

    return 0;
}
