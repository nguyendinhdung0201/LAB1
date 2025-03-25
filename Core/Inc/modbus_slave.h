/*
 * modbus_slave.h
 *
 *  Created on: Dec 24, 2024
 *      Author: PC
 */

#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_
#include "main.h"
#include "string.h"

// Định nghĩa kích thước buffer
#define MODBUS_RX_BUFFER_SIZE 256
#define MODBUS_TX_BUFFER_SIZE 256

// Định nghĩa địa chỉ slave
#define SLAVE_ADDRESS 0x04

// Định nghĩa function codes
#define READ_COILS          0x01
#define READ_DISCRETE       0x02
#define READ_HOLDING        0x03
#define READ_INPUT          0x04
#define WRITE_SINGLE_COIL   0x05
#define WRITE_SINGLE_REG    0x06
#define WRITE_MULTI_COILS   0x0F
#define WRITE_MULTI_REGS    0x10

// Định nghĩa số lượng thanh ghi tối đa
#define MAX_HOLDING_REGS    100
#define MAX_INPUT_REGS      100
#define MAX_COILS           100
#define MAX_DISCRETE        100
// Thêm định nghĩa trạng thái

extern uint8_t check_report_data;
typedef enum {
    MODBUS_READY,
    MODBUS_PROCESSING,
    MODBUS_TRANSMITTING
} ModbusState;
// Cấu trúc dữ liệu Modbus
typedef struct {
    UART_HandleTypeDef* huart;    // Handle UART
    GPIO_TypeDef* dePort;         // PORT của chân DE/RE
    uint16_t dePin;               // PIN của chân DE/RE

    uint8_t rxBuffer[MODBUS_RX_BUFFER_SIZE];
    uint8_t txBuffer[MODBUS_TX_BUFFER_SIZE];
    volatile uint16_t rxCount;

    uint16_t holdingRegs[MAX_HOLDING_REGS];
    uint16_t inputRegs[MAX_INPUT_REGS];
    uint8_t coils[MAX_COILS];
    uint8_t discreteInputs[MAX_DISCRETE];
    volatile ModbusState state;  // Thêm trạng thái

    uint32_t lastResetTime;
} ModbusHandle;

// Hàm khởi tạo
//void Modbus_Init(ModbusHandle* modbus, UART_HandleTypeDef* huart,
//                 GPIO_TypeDef* dePort, uint16_t dePin);
void Modbus_Init(ModbusHandle* modbus, UART_HandleTypeDef* huart);

// Hàm xử lý dữ liệu nhận được
void Modbus_ProcessData(ModbusHandle* modbus);

// Hàm callback khi nhận dữ liệu
void Modbus_UartRxCpltCallback(ModbusHandle* modbus, uint16_t Size);
void Data_Write();
#endif /* INC_MODBUS_SLAVE_H_ */
