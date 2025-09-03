#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

// Modbus Function Codes
#define MODBUS_FC_READ_COILS                    0x01
#define MODBUS_FC_READ_DISCRETE                 0x02
#define MODBUS_FC_READ_HOLDING_REG              0x03
#define MODBUS_FC_READ_INPUT_REG                0x04
#define MODBUS_FC_WRITE_SINGLE_COIL             0x05
#define MODBUS_FC_WRITE_SINGLE_REG              0x06
#define MODBUS_FC_WRITE_MULTIPLE_COIL           0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REG            0x10

// Modbus Exception Codes
#define MODBUS_EX_NONE                0x00
#define MODBUS_EX_ILLEGAL_FUNCTION    0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDR   0x02
#define MODBUS_EX_ILLEGAL_DATA_VAL    0x03
#define MODBUS_EX_SLAVE_DEVICE_FAIL   0x04

// Modbus Process Packet Error Codes
typedef enum {
    MODBUS_PROCESS_SUCCESS = 0,           // Packet processed successfully
    MODBUS_PROCESS_ERROR_INVALID_LENGTH,  // Packet too short (less than 4 bytes)
    MODBUS_PROCESS_ERROR_INVALID_CRC,     // CRC validation failed
    MODBUS_PROCESS_ERROR_WRONG_SLAVE_ID,  // Packet not for this slave
    MODBUS_PROCESS_ERROR_INVALID_FUNCTION, // Unsupported function code
    MODBUS_PROCESS_ERROR_INVALID_DATA,    // Invalid data values in packet
    MODBUS_PROCESS_ERROR_BUFFER_OVERFLOW, // Response would exceed buffer size
    MODBUS_PROCESS_ERROR_CALLBACK_FAILED, // Callback function failed
    MODBUS_PROCESS_ERROR_UNKNOWN          // Unknown error
} modbus_process_error_t;

#define MODBUS_BUFFER_SIZE 256

// Callback function types
typedef uint8_t (*modbus_write_callback_t)(uint8_t *buffer, uint16_t length);
typedef uint16_t (*modbus_holding_register_read_callback_t)(uint16_t address);
typedef void (*modbus_holding_register_write_callback_t)(uint16_t address, uint16_t value);
typedef uint16_t (*modbus_input_register_read_callback_t)(uint16_t address);

typedef struct {
    uint8_t slave_id;
    modbus_write_callback_t write_packet_callback;
    modbus_holding_register_read_callback_t holding_register_read_callback;
    modbus_holding_register_write_callback_t holding_register_write_callback;
    modbus_input_register_read_callback_t input_register_read_callback;
    
    // Instance-specific buffers
    uint8_t rx_buffer[MODBUS_BUFFER_SIZE];  // 256 bytes per instance
    uint8_t tx_buffer[MODBUS_BUFFER_SIZE];  // 256 bytes per instance
} modbus_t;

// Function declarations
void modbus_init(modbus_t *modbus, uint8_t slave_id, 
                 modbus_write_callback_t write_packet_callback,
                 modbus_holding_register_read_callback_t holding_register_read_callback,
                 modbus_holding_register_write_callback_t holding_register_write_callback,
                 modbus_input_register_read_callback_t input_register_read_callback);

void modbus_clear_buffers(modbus_t *modbus);

modbus_process_error_t modbus_process_packet(modbus_t *modbus, uint8_t *data, uint16_t data_len);
uint8_t modbus_read_packet(uint8_t *buffer, uint16_t index, uint16_t length);

// Utility functions
uint16_t modbus_crc16(uint8_t *data, uint16_t len);
bool modbus_validate_crc(uint8_t *data, uint16_t data_len);

#endif // MODBUS_SLAVE_H