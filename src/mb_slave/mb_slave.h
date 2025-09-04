#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

// Modbus Function Codes
#define MODBUS_FC_READ_HOLDING_REG              0x03
#define MODBUS_FC_READ_INPUT_REG                0x04
#define MODBUS_FC_WRITE_SINGLE_REG              0x06
#define MODBUS_FC_WRITE_MULTIPLE_REG            0x10

// Modbus Exception Codes
#define MODBUS_EX_NONE                0x00
#define MODBUS_EX_ILLEGAL_FUNCTION    0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDR   0x02
#define MODBUS_EX_ILLEGAL_DATA_VAL    0x03

// Modbus Process Packet Error Codes
typedef enum {
    MODBUS_PROCESS_SUCCESS = 0,           // Packet processed successfully
    MODBUS_PROCESS_ERROR_INVALID_CRC,     // CRC validation failed
    MODBUS_PROCESS_ERROR_WRONG_SLAVE_ID,  // Packet not for this slave
    MODBUS_PROCESS_ERROR_INVALID_FUNCTION, // Unsupported function code
    MODBUS_PROCESS_ERROR_INVALID_DATA     // Invalid data values in packet
} modbus_process_error_t;

#define MODBUS_BUFFER_SIZE 256

// Callback function types
// WARNING: These callbacks are called from the Modbus processing thread and will block it.
// Keep callback execution time minimal to avoid affecting Modbus response timing.
typedef uint16_t (*modbus_write_callback_t)(uint8_t *buffer, uint16_t length);
typedef void (*modbus_holding_register_write_callback_t)(uint16_t address, uint16_t value);
typedef void (*modbus_holding_register_read_multiple_callback_t)(uint16_t start_address, uint16_t quantity, uint16_t *values);
typedef void (*modbus_holding_register_write_multiple_callback_t)(uint16_t start_address, uint16_t quantity, const uint16_t *values);
typedef void (*modbus_input_register_read_multiple_callback_t)(uint16_t start_address, uint16_t quantity, uint16_t *values);


//TODO: Add more callbacks for other function codes
typedef struct {
    modbus_write_callback_t write_packet_callback; // For writing packet to serial
    modbus_holding_register_write_callback_t holding_register_write_callback;
    modbus_holding_register_read_multiple_callback_t holding_register_read_multiple_callback;
    modbus_holding_register_write_multiple_callback_t holding_register_write_multiple_callback;
    modbus_input_register_read_multiple_callback_t input_register_read_multiple_callback;
} modbus_callbacks_t;

typedef struct {
    uint8_t slave_id;
    modbus_callbacks_t callbacks;
    uint16_t max_holding_register_address;  // Maximum holding register address (0 to max_holding_register_address)
    uint16_t max_input_register_address;    // Maximum input register address (0 to max_input_register_address)
    
    // Instance-specific buffer
    uint8_t tx_buffer[MODBUS_BUFFER_SIZE];  // 256 bytes per instance
} modbus_t;

// Function declarations
void modbus_init(modbus_t *modbus, uint8_t slave_id, uint16_t max_holding_register_address, uint16_t max_input_register_address, const modbus_callbacks_t *callbacks);

void modbus_clear_buffers(modbus_t *modbus);

modbus_process_error_t modbus_process_packet(modbus_t *modbus, uint8_t *data, uint16_t data_len);

// Utility functions
uint16_t modbus_crc16(uint8_t *data, uint16_t len);
bool modbus_validate_crc(uint8_t *data, uint16_t data_len);

#endif // MODBUS_SLAVE_H