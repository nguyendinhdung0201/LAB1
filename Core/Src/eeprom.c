/*
 * eeprom.c
 *
 *  Created on: Nov 16, 2024
 *      Author: PC
 */
#include "eeprom.h"
AT24C01_HandleTypeDef at24c01;
// Khởi tạo AT24C01
HAL_StatusTypeDef AT24C01_Init(AT24C01_HandleTypeDef *handle, I2C_HandleTypeDef *hi2c)
{
    handle->hi2c = hi2c;
    handle->deviceAddress = AT24C01_ADDRESS;
    handle->timeout = 1000; // Timeout 1 giây

    // Kiểm tra device có tồn tại không
    if (HAL_I2C_IsDeviceReady(handle->hi2c, handle->deviceAddress, 2, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Ghi 1 byte vào địa chỉ cụ thể
HAL_StatusTypeDef AT24C01_WriteByte(AT24C01_HandleTypeDef *handle, uint8_t memAddress, uint8_t data)
{
    uint8_t buffer[2];
    buffer[0] = memAddress;
    buffer[1] = data;

    // Ghi dữ liệu
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c,
                                                      handle->deviceAddress,
                                                      buffer,
                                                      2,
                                                      handle->timeout);

    // Đợi cho quá trình ghi hoàn tất
    HAL_Delay(5);

    return status;
}

// Đọc 1 byte từ địa chỉ cụ thể
HAL_StatusTypeDef AT24C01_ReadByte(AT24C01_HandleTypeDef *handle, uint8_t memAddress, uint8_t *data)
{
    // Gửi địa chỉ muốn đọc
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c,
                                                      handle->deviceAddress,
                                                      &memAddress,
                                                      1,
                                                      handle->timeout);
    if (status != HAL_OK) {
        return status;
    }

    // Đọc dữ liệu
    return HAL_I2C_Master_Receive(handle->hi2c,
                                 handle->deviceAddress,
                                 data,
                                 1,
                                 handle->timeout);
}

// Ghi nhiều bytes liên tiếp
HAL_StatusTypeDef AT24C01_WriteBytes(AT24C01_HandleTypeDef *handle, uint8_t memAddress, uint8_t *data, uint8_t size)
{
    if (size > 8) { // AT24C01 chỉ hỗ trợ page write tối đa 8 bytes
        return HAL_ERROR;
    }

    uint8_t buffer[9]; // 1 byte địa chỉ + tối đa 8 bytes dữ liệu
    buffer[0] = memAddress;
    for (uint8_t i = 0; i < size; i++) {
        buffer[i + 1] = data[i];
    }

    // Ghi dữ liệu
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c,
                                                      handle->deviceAddress,
                                                      buffer,
                                                      size + 1,
                                                      handle->timeout);

    // Đợi cho quá trình ghi hoàn tất
    HAL_Delay(5);

    return status;
}

// Đọc nhiều bytes liên tiếp
HAL_StatusTypeDef AT24C01_ReadBytes(AT24C01_HandleTypeDef *handle, uint8_t memAddress, uint8_t *data, uint8_t size)
{
    // Gửi địa chỉ muốn đọc
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c,
                                                      handle->deviceAddress,
                                                      &memAddress,
                                                      1,
                                                      handle->timeout);
    if (status != HAL_OK) {
        return status;
    }

    // Đọc dữ liệu
    return HAL_I2C_Master_Receive(handle->hi2c,
                                 handle->deviceAddress,
                                 data,
                                 size,
                                 handle->timeout);
}

// Xóa toàn bộ EEPROM (ghi 0xFF vào tất cả các ô nhớ)
HAL_StatusTypeDef AT24C01_EraseAll(AT24C01_HandleTypeDef *handle)
{
    uint8_t data = 0xFF;
    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t addr = 0; addr < 128; addr++) {
        status = AT24C01_WriteByte(handle, addr, data);
        if (status != HAL_OK) {
            return status;
        }
    }

    return status;
}

HAL_StatusTypeDef AT24C01_Write_2_Bytes(AT24C01_HandleTypeDef *handle, uint8_t addr1, uint8_t addr2, int16_t data)
{
	HAL_StatusTypeDef status;
	uint8_t high_Byte = (data >> 8) & 0xFF;
	uint8_t low_Byte = data & 0xFF;
    // Ghi byte cao
    status = AT24C01_WriteByte(handle, addr1, high_Byte);
    if(status != HAL_OK) {
        return status;
    }
    // Delay để đảm bảo chu trình ghi hoàn tất (thường 5-10ms)
    HAL_Delay(5);
    // Ghi byte thấp
    status = AT24C01_WriteByte(handle, addr2, low_Byte);
    if(status != HAL_OK) {
        return status;
    }

    // Delay cho chu trình ghi cuối
    HAL_Delay(5);

    return HAL_OK;
}

HAL_StatusTypeDef AT24C01_Write_2_Bytes_u(AT24C01_HandleTypeDef *handle, uint8_t addr1, uint8_t addr2, uint16_t data)
{
	HAL_StatusTypeDef status;
	uint8_t high_Byte = (data >> 8);
	uint8_t low_Byte = data & 0xFF;
    // Ghi byte cao
    status = AT24C01_WriteByte(handle, addr1, high_Byte);
    if(status != HAL_OK) {
        return status;
    }
    // Delay để đảm bảo chu trình ghi hoàn tất (thường 5-10ms)
    HAL_Delay(5);
    // Ghi byte thấp
    status = AT24C01_WriteByte(handle, addr2, low_Byte);
    if(status != HAL_OK) {
        return status;
    }

    // Delay cho chu trình ghi cuối
    HAL_Delay(5);

    return HAL_OK;
}
//int16_t AT24C01_Read_2_Bytes(AT24C01_HandleTypeDef *handle, uint8_t addr1, uint8_t addr2)
//{
//	uint8_t high_Byte, low_Byte;
//	if(AT24C01_ReadByte(handle, addr1, &high_Byte) != HAL_OK){
//		return -1;
//	}
//	if(AT24C01_ReadByte(handle, addr2, &low_Byte) != HAL_OK){
//		return -1;
//	}
//	return (int16_t)((high_Byte << 8) | low_Byte);
//}
int16_t AT24C01_Read_2_Bytes(AT24C01_HandleTypeDef *handle, uint8_t startAddr)
{
    uint8_t bytes[2];
    HAL_StatusTypeDef status;

    // Gửi địa chỉ bắt đầu đọc
    status = HAL_I2C_Master_Transmit(handle->hi2c,
                                   handle->deviceAddress,
                                   &startAddr,
                                   1,
                                   handle->timeout);
    if (status != HAL_OK) {
        return -1;
    }

    // Đọc 2 byte liên tiếp
    status = HAL_I2C_Master_Receive(handle->hi2c,
                                  handle->deviceAddress,
                                  bytes,
                                  2,
                                  handle->timeout);
    if (status != HAL_OK) {
        return -1;
    }

    // Ghép 2 byte thành int16_t
    return (int16_t)((bytes[0] << 8) | bytes[1]);
}
uint16_t AT24C01_Read_2_Bytes_u(AT24C01_HandleTypeDef *handle, uint8_t startAddr)
{
    uint8_t bytes[2];
    HAL_StatusTypeDef status;

    // Gửi địa chỉ bắt đầu đọc
    status = HAL_I2C_Master_Transmit(handle->hi2c,
                                   handle->deviceAddress,
                                   &startAddr,
                                   1,
                                   handle->timeout);
    if (status != HAL_OK) {
        return -1;
    }

    // Đọc 2 byte liên tiếp
    status = HAL_I2C_Master_Receive(handle->hi2c,
                                  handle->deviceAddress,
                                  bytes,
                                  2,
                                  handle->timeout);
    if (status != HAL_OK) {
        return -1;
    }

    // Ghép 2 byte thành int16_t
    return (uint16_t)((bytes[0] << 8) | bytes[1]);
}
