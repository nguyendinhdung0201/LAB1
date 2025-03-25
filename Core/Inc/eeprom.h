/*
 * eeprom.h
 *
 *  Created on: Nov 16, 2024
 *      Author: PC
 */

#ifndef EEPROM_H_
#define EEPROM_H_
#include "main.h"
#include <string.h>

// Định nghĩa địa chỉ I2C của AT24C01
#define AT24C01_ADDRESS   0xA0 // Địa chỉ mặc định: 1010 000x (x là bit R/W)

// Cấu trúc cho AT24C01
typedef struct {
    I2C_HandleTypeDef *hi2c;     // Handle của I2C
    uint16_t deviceAddress;      // Địa chỉ I2C của device
    uint32_t timeout;            // Timeout cho các thao tác I2C
} AT24C01_HandleTypeDef;
extern AT24C01_HandleTypeDef at24c01;
// Các hàm khởi tạo và điều khiển
HAL_StatusTypeDef AT24C01_Init(AT24C01_HandleTypeDef *handle, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef AT24C01_WriteByte(AT24C01_HandleTypeDef *handle, uint8_t memAddress, uint8_t data);
HAL_StatusTypeDef AT24C01_ReadByte(AT24C01_HandleTypeDef *handle, uint8_t memAddress, uint8_t *data);
HAL_StatusTypeDef AT24C01_WriteBytes(AT24C01_HandleTypeDef *handle, uint8_t memAddress, uint8_t *data, uint8_t size);
HAL_StatusTypeDef AT24C01_ReadBytes(AT24C01_HandleTypeDef *handle, uint8_t memAddress, uint8_t *data, uint8_t size);
HAL_StatusTypeDef AT24C01_EraseAll(AT24C01_HandleTypeDef *handle);
HAL_StatusTypeDef AT24C01_Write_2_Bytes(AT24C01_HandleTypeDef *handle, uint8_t addr1, uint8_t addr2, int16_t data);
HAL_StatusTypeDef AT24C01_Write_2_Bytes_u(AT24C01_HandleTypeDef *handle, uint8_t addr1, uint8_t addr2, uint16_t data);
//int16_t AT24C01_Read_2_Bytes(AT24C01_HandleTypeDef *handle, uint8_t addr1, uint8_t addr2);
int16_t AT24C01_Read_2_Bytes(AT24C01_HandleTypeDef *handle, uint8_t startAddr);
uint16_t AT24C01_Read_2_Bytes_u(AT24C01_HandleTypeDef *handle, uint8_t startAddr);
#endif /* INC_EEPROM_H_ */
