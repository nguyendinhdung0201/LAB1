/*
 * modbus_slave.c
 *
 *  Created on: Dec 24, 2024
 *      Author: PC
 */
#include "modbus_slave.h"
#include "eeprom.h"
// Định nghĩa các exception codes
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION    0x01
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS     0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE  0x03
uint8_t check_report_data = 0;
uint16_t holdingRegs_cpy[MAX_HOLDING_REGS];
volatile uint16_t num_reg = 0;
volatile uint8_t check_feedback_data = 0;
// Tính CRC16
static uint16_t Modbus_CRC16(uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Modbus_Init(ModbusHandle* modbus, UART_HandleTypeDef* huart) {
    modbus->huart = huart;
    modbus->state = MODBUS_READY;  // Khởi tạo trạng thái
    // Khởi tạo các giá trị mặc định
    memset(modbus->holdingRegs, 0, sizeof(modbus->holdingRegs));
    memset(modbus->inputRegs, 0, sizeof(modbus->inputRegs));
    memset(modbus->coils, 0, sizeof(modbus->coils));
    memset(modbus->discreteInputs, 0, sizeof(modbus->discreteInputs));
    HAL_UARTEx_ReceiveToIdle_DMA(modbus->huart, modbus->rxBuffer, MODBUS_RX_BUFFER_SIZE);
}

// Hàm gửi response
static void Modbus_SendResponse(ModbusHandle* modbus, uint16_t length) {
    uint16_t crc = Modbus_CRC16(modbus->txBuffer, length);
    modbus->txBuffer[length] = crc & 0xFF;
    modbus->txBuffer[length + 1] = (crc >> 8) & 0xFF;
    modbus->state = MODBUS_TRANSMITTING;  // Đánh dấu đang truyền
    HAL_UART_Transmit_DMA(modbus->huart, modbus->txBuffer, length + 2);
}

// Xử lý đọc holding registers
static void Modbus_HandleReadHolding(ModbusHandle* modbus, uint16_t address, uint16_t quantity) {
    if (address + quantity <= MAX_HOLDING_REGS) {
        modbus->txBuffer[2] = quantity * 2;  // Byte count

        for (int i = 0; i < quantity; i++) {
            modbus->txBuffer[3 + i*2] = (modbus->holdingRegs[address + i] >> 8) & 0xFF;
            modbus->txBuffer[4 + i*2] = modbus->holdingRegs[address + i] & 0xFF;
        }

        Modbus_SendResponse(modbus, 3 + quantity * 2);
    }
}
// Xử lý ghi single holding register
static void Modbus_HandleWriteSingleReg(ModbusHandle* modbus, uint16_t address, uint16_t value) {
    if (address < MAX_HOLDING_REGS) {
        modbus->holdingRegs[address] = value;
        memcpy(modbus->txBuffer, modbus->rxBuffer, 6);  // Copy request làm response
        Modbus_SendResponse(modbus, 6);
    }
}
static void Modbus_HandleWriteMultipleRegs(ModbusHandle* modbus, uint16_t address, uint16_t quantity, uint16_t* values){
	if (address + quantity <= MAX_HOLDING_REGS){
        // Ghi giá trị vào các registers
        for (uint16_t i = 0; i < quantity; i++) {
            modbus->holdingRegs[address + i] = values[i];
        }
        // Chuẩn bị response
        // Format response:
        // Byte 0: Slave address (đã được copy)
        // Byte 1: Function code (đã được copy)
        // Byte 2-3: Starting address
        // Byte 4-5: Number of registers written
        modbus->txBuffer[2] = (uint8_t)(address >> 8);    // High byte of address
        modbus->txBuffer[3] = (uint8_t)(address & 0xFF);  // Low byte of address
        modbus->txBuffer[4] = (uint8_t)(quantity >> 8);   // High byte of quantity
        modbus->txBuffer[5] = (uint8_t)(quantity & 0xFF); // Low byte of quantity

        // Tính toán CRC và thêm vào cuối response
        uint16_t crc = Modbus_CRC16(modbus->txBuffer, 6);
        modbus->txBuffer[6] = (uint8_t)(crc & 0xFF);      // Low byte of CRC
        modbus->txBuffer[7] = (uint8_t)(crc >> 8);        // High byte of CRC

        // Gửi response với độ dài 8 bytes
        Modbus_SendResponse(modbus, 8);
	}
    else {
        // Gửi exception response nếu địa chỉ không hợp lệ
        modbus->txBuffer[1] |= 0x80;  // Set MSB của function code để chỉ ra exception
        modbus->txBuffer[2] = MODBUS_EXCEPTION_ILLEGAL_ADDRESS;

        // Tính CRC cho exception response
        uint16_t crc = Modbus_CRC16(modbus->txBuffer, 3);
        modbus->txBuffer[3] = (uint8_t)(crc & 0xFF);
        modbus->txBuffer[4] = (uint8_t)(crc >> 8);

        // Gửi exception response
        Modbus_SendResponse(modbus, 5);
    }
}
// Xử lý dữ liệu nhận được
void Modbus_ProcessData(ModbusHandle* modbus) {
	// Nếu đang bận, bỏ qua request mới
	if (modbus->state != MODBUS_READY) {
	    return;
	}
    // Kiểm tra địa chỉ slave
    if (modbus->rxBuffer[0] != SLAVE_ADDRESS) return;

    // Kiểm tra CRC
    uint16_t receivedCrc = (modbus->rxBuffer[modbus->rxCount - 1] << 8) |
                           modbus->rxBuffer[modbus->rxCount - 2];
    uint16_t calculatedCrc = Modbus_CRC16(modbus->rxBuffer, modbus->rxCount - 2);
    if (receivedCrc != calculatedCrc) return;
    modbus->state = MODBUS_PROCESSING;  // Đánh dấu đang xử lý
    // Copy địa chỉ và function code vào buffer phản hồi
    modbus->txBuffer[0] = modbus->rxBuffer[0];  // Slave address
    modbus->txBuffer[1] = modbus->rxBuffer[1];  // Function code

    uint8_t functionCode = modbus->rxBuffer[1];
    uint16_t address = (modbus->rxBuffer[2] << 8) | modbus->rxBuffer[3];
    uint16_t quantity = (modbus->rxBuffer[4] << 8) | modbus->rxBuffer[5];

    switch (functionCode) {
        case READ_HOLDING:
        	check_report_data = 0;
            Modbus_HandleReadHolding(modbus, address, quantity);
            break;

        case WRITE_SINGLE_REG:
        	check_report_data = 0;
            Modbus_HandleWriteSingleReg(modbus, address, quantity);
            break;
        case WRITE_MULTI_REGS:
            check_report_data = 0;
            uint8_t byteCount = modbus->rxBuffer[6];
            if (byteCount != quantity * 2) {
                modbus->txBuffer[1] |= 0x80;
                modbus->txBuffer[2] = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
                Modbus_SendResponse(modbus, 3);
                break;
            }

            if (address + quantity > MAX_HOLDING_REGS) {
                modbus->txBuffer[1] |= 0x80;
                modbus->txBuffer[2] = MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
                Modbus_SendResponse(modbus, 3);
                break;
            }
            uint16_t values[MAX_HOLDING_REGS];
            for (uint16_t i = 0; i < quantity; i++) {
                values[i] = (modbus->rxBuffer[7 + i*2] << 8) |
                           modbus->rxBuffer[8 + i*2];
            }
            Modbus_HandleWriteMultipleRegs(modbus, address, quantity, values);
            check_feedback_data = 1;
//            num_reg = quantity;
            memcpy(holdingRegs_cpy,modbus->holdingRegs,sizeof(int16_t) * 2);
            break;
        default:
            // Exception response
            modbus->txBuffer[1] |= 0x80;  // Set MSB của function code
            modbus->txBuffer[2] = 0x01;   // Exception code: Illegal function
            Modbus_SendResponse(modbus, 3);
            break;
    }
}
// Callback khi nhận dữ liệu
void Modbus_UartRxCpltCallback(ModbusHandle* modbus, uint16_t Size) {
    modbus->rxCount = Size;
    Modbus_ProcessData(modbus);
    HAL_UARTEx_ReceiveToIdle_DMA(modbus->huart, modbus->rxBuffer, MODBUS_RX_BUFFER_SIZE);
}
void Data_Write(){
	if(check_feedback_data == 1){
		if(nhiet_do_bat_lam_mat != holdingRegs_cpy[30] || nhiet_do_tat_lam_mat !=  holdingRegs_cpy[31]){
			AT24C01_Write_2_Bytes(&at24c01, 0, 1, holdingRegs_cpy[30]);
			AT24C01_Write_2_Bytes(&at24c01, 1, 2, holdingRegs_cpy[31]);
			check_feedback_data = 0;
			nhiet_do_bat_lam_mat = holdingRegs_cpy[30];
			nhiet_do_tat_lam_mat = holdingRegs_cpy[31];
		}
	}
}
