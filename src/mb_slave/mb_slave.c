#include "mb_slave.h"
#include "ch.h"
#include "hal.h"
#include <string.h>

// Private Types
typedef struct {
    uint8_t slave_id;
    uint8_t function_code;
    uint16_t start_address;
    uint16_t quantity;
} modbus_request_t;

typedef struct {
    uint8_t slave_id;
    uint8_t function_code;
    uint8_t exception_code;
    uint16_t data_len;
} modbus_response_t;

// Private Functions
static void modbus_send_response(modbus_t *modbus, modbus_response_t *response);
static void modbus_send_exception_response(modbus_t *modbus, uint8_t slave_id, uint8_t function_code, uint8_t exception_code);
static bool modbus_validate_frame_length(uint8_t function_code, uint16_t len);
static bool modbus_validate_holding_register_address(modbus_t *modbus, uint16_t start_address, uint16_t quantity);
static bool modbus_validate_input_register_address(modbus_t *modbus, uint16_t start_address, uint16_t quantity);

void modbus_init(modbus_t *modbus, uint8_t slave_id, uint16_t max_holding_register_address, uint16_t max_input_register_address, const modbus_callbacks_t *callbacks) {
    modbus->slave_id = slave_id;
    modbus->max_holding_register_address = max_holding_register_address;
    modbus->max_input_register_address = max_input_register_address;
    
    // Copy callbacks structure
    if (callbacks) {
        modbus->callbacks = *callbacks;
    } else {
        // Initialize all callbacks to NULL if no callbacks provided
        memset(&modbus->callbacks, 0, sizeof(modbus_callbacks_t));
    }
    
    // Initialize instance-specific buffer
    memset(modbus->tx_buffer, 0, sizeof(modbus->tx_buffer));
}

void modbus_clear_buffers(modbus_t *modbus) {
    if (modbus) {
        memset(modbus->tx_buffer, 0, sizeof(modbus->tx_buffer));
    }
}

// Send Modbus exception response
static void modbus_send_exception_response(modbus_t *modbus, uint8_t slave_id, uint8_t function_code, uint8_t exception_code) {
    modbus_response_t response;
    
    response.slave_id = slave_id;
    response.function_code = function_code;
    response.exception_code = exception_code;
    response.data_len = 0;
    
    modbus_send_response(modbus, &response);
}

// Validate Modbus frame length according to standard
static bool modbus_validate_frame_length(uint8_t function_code, uint16_t len) {
    switch (function_code) {
        case MODBUS_FC_READ_HOLDING_REG:
        case MODBUS_FC_READ_INPUT_REG:
            return len >= 8; // 6 data + 2 CRC
        case MODBUS_FC_WRITE_SINGLE_REG:
            return len >= 8; // 6 data + 2 CRC
        case MODBUS_FC_WRITE_MULTIPLE_REG:
            return len >= 9; // 7+ data + 2 CRC (minimum)
        default:
            return false; // Unsupported function code
    }
}

// Process Modbus request packets
modbus_process_error_t modbus_process_packet(modbus_t *modbus, uint8_t *data, uint16_t len) {
    // Parse basic request info to check slave ID
    
    uint8_t slave_id = data[0];
    uint8_t function_code = data[1];
    
    // Check if this request is for us BEFORE doing expensive operations
    if (slave_id != modbus->slave_id && slave_id != 0) {
        return MODBUS_PROCESS_ERROR_WRONG_SLAVE_ID; // Not for us, don't respond
    }
    
    // Validate function code range (Standard Modbus RTU check)
    if (function_code < 1 || function_code > 127) {
        modbus_send_exception_response(modbus, slave_id, function_code, MODBUS_EX_ILLEGAL_FUNCTION);
        return MODBUS_PROCESS_ERROR_INVALID_FUNCTION;
    }
    
    // Validate frame length (Standard Modbus RTU check)
    if (!modbus_validate_frame_length(function_code, len)) {
        modbus_send_exception_response(modbus, slave_id, function_code, MODBUS_EX_ILLEGAL_DATA_VAL);
        return MODBUS_PROCESS_ERROR_INVALID_DATA;
    }
    
    // Now that we know it's for us, validate CRC (Standard Modbus RTU check)
    if (!modbus_validate_crc(data, len)) {
        // Send exception response for invalid CRC (only if it's for us)
        modbus_send_exception_response(modbus, slave_id, function_code, MODBUS_EX_ILLEGAL_DATA_VAL);
        return MODBUS_PROCESS_ERROR_INVALID_CRC;
    }
    
    modbus_request_t request;
    modbus_response_t response;
    
    // Parse Modbus request
    request.slave_id = slave_id;
    request.function_code = function_code;
    request.start_address = (data[2] << 8) | data[3];
    
    // Initialize response
    response.slave_id = modbus->slave_id;
    response.function_code = request.function_code;
    response.exception_code = MODBUS_EX_NONE;
    response.data_len = 0;
    
    // Process based on function code
    switch (request.function_code) {
        case MODBUS_FC_READ_HOLDING_REG: {
            if (len >= 6) {
                request.quantity = (data[4] << 8) | data[5];
                
                // Validate holding register address range
                if (!modbus_validate_holding_register_address(modbus, request.start_address, request.quantity)) {
                    response.exception_code = MODBUS_EX_ILLEGAL_DATA_ADDR;
                    break;
                }
                
                if (request.quantity >= 1 && request.quantity <= 125) {
                    response.data_len = request.quantity * 2;
                    
                    
                    // Store response data directly in tx_buffer
                    modbus->tx_buffer[0] = response.slave_id;
                    modbus->tx_buffer[1] = response.function_code;
                    modbus->tx_buffer[2] = response.data_len; // Byte count
                    
                    // Call callback directly from struct
                    uint16_t values[125]; // Max quantity is 125
                    if (modbus->callbacks.holding_register_read_multiple_callback) {
                        modbus->callbacks.holding_register_read_multiple_callback(request.start_address, request.quantity, values);
                    }
                    
                    // Copy values from callback
                    for (int i = 0; i < request.quantity; i++) {
                        modbus->tx_buffer[3 + i * 2] = (values[i] >> 8) & 0xFF;
                        modbus->tx_buffer[4 + i * 2] = values[i] & 0xFF;
                    }
                } else {
                    response.exception_code = MODBUS_EX_ILLEGAL_DATA_VAL;
                }
            } else {
                response.exception_code = MODBUS_EX_ILLEGAL_DATA_VAL;
            }
            break;
        }
        
        case MODBUS_FC_READ_INPUT_REG: {
            if (len >= 6) {
                request.quantity = (data[4] << 8) | data[5];
                
                // Validate input register address range
                if (!modbus_validate_input_register_address(modbus, request.start_address, request.quantity)) {
                    response.exception_code = MODBUS_EX_ILLEGAL_DATA_ADDR;
                    break;
                }
                
                if (request.quantity >= 1 && request.quantity <= 125) {
                    response.data_len = request.quantity * 2;
                    
                    
                    // Store response data directly in tx_buffer
                    modbus->tx_buffer[0] = response.slave_id;
                    modbus->tx_buffer[1] = response.function_code;
                    modbus->tx_buffer[2] = response.data_len; // Byte count
                    
                    // Call callback directly from struct
                    uint16_t values[125]; // Max quantity is 125
                    if (modbus->callbacks.input_register_read_multiple_callback) {
                        modbus->callbacks.input_register_read_multiple_callback(request.start_address, request.quantity, values);
                    }
                    
                    // Copy values from callback
                    for (int i = 0; i < request.quantity; i++) {
                        modbus->tx_buffer[3 + i * 2] = (values[i] >> 8) & 0xFF;
                        modbus->tx_buffer[4 + i * 2] = values[i] & 0xFF;
                    }
                } else {
                    response.exception_code = MODBUS_EX_ILLEGAL_DATA_VAL;
                }
            } else {
                response.exception_code = MODBUS_EX_ILLEGAL_DATA_VAL;
            }
            break;
        }
        
        case MODBUS_FC_WRITE_SINGLE_REG: {
            if (len >= 6) {
                uint16_t reg_value = (data[4] << 8) | data[5];
                
                // Validate holding register address range (quantity = 1 for single register)
                if (!modbus_validate_holding_register_address(modbus, request.start_address, 1)) {
                    response.exception_code = MODBUS_EX_ILLEGAL_DATA_ADDR;
                    break;
                }
                
                // Call callback directly from struct
                if (modbus->callbacks.holding_register_write_callback) {
                    modbus->callbacks.holding_register_write_callback(request.start_address, reg_value);
                }
                
                // Echo back the write request
                response.data_len = 4;
                
                
                // Store response data directly in tx_buffer
                modbus->tx_buffer[0] = response.slave_id;
                modbus->tx_buffer[1] = response.function_code;
                modbus->tx_buffer[2] = (request.start_address >> 8) & 0xFF;
                modbus->tx_buffer[3] = request.start_address & 0xFF;
                modbus->tx_buffer[4] = (reg_value >> 8) & 0xFF;
                modbus->tx_buffer[5] = reg_value & 0xFF;
            } else {
                response.exception_code = MODBUS_EX_ILLEGAL_DATA_VAL;
            }
            break;
        }
        
        case MODBUS_FC_WRITE_MULTIPLE_REG: {
            if (len >= 7) {
                request.quantity = (data[4] << 8) | data[5];
                uint8_t byte_count = data[6];
                
                // Validate holding register address range
                if (!modbus_validate_holding_register_address(modbus, request.start_address, request.quantity)) {
                    response.exception_code = MODBUS_EX_ILLEGAL_DATA_ADDR;
                    break;
                }
                
                if (request.quantity >= 1 && request.quantity <= 123 && 
                    len >= 7 + byte_count + 2 && byte_count == request.quantity * 2) {
                    
                    // Extract values from request data
                    uint16_t values[123]; // Max quantity is 123
                    for (int i = 0; i < request.quantity; i++) {
                        values[i] = (data[7 + i * 2] << 8) | data[8 + i * 2];
                    }
                    
                    // Call callback directly from struct
                    if (modbus->callbacks.holding_register_write_multiple_callback) {
                        modbus->callbacks.holding_register_write_multiple_callback(request.start_address, request.quantity, values);
                    }
                    
                    // Echo back the write request
                    response.data_len = 4;
                    
                    
                    // Store response data directly in tx_buffer
                    modbus->tx_buffer[0] = response.slave_id;
                    modbus->tx_buffer[1] = response.function_code;
                    modbus->tx_buffer[2] = (request.start_address >> 8) & 0xFF;
                    modbus->tx_buffer[3] = request.start_address & 0xFF;
                    modbus->tx_buffer[4] = (request.quantity >> 8) & 0xFF;
                    modbus->tx_buffer[5] = request.quantity & 0xFF;
                } else {
                    response.exception_code = MODBUS_EX_ILLEGAL_DATA_VAL;
                }
            } else {
                response.exception_code = MODBUS_EX_ILLEGAL_DATA_VAL;
            }
            break;
        }
        
        default:
            response.exception_code = MODBUS_EX_ILLEGAL_FUNCTION;
            break;
    }
    
    // Send response
    modbus_send_response(modbus, &response);
    return MODBUS_PROCESS_SUCCESS; // Successfully processed
}

// Send Modbus response
static void modbus_send_response(modbus_t *modbus, modbus_response_t *response) {
    uint16_t index = 0;
    
    if (response->exception_code != MODBUS_EX_NONE) {
        // Exception response - build from scratch
        memset(modbus->tx_buffer, 0, sizeof(modbus->tx_buffer));
        modbus->tx_buffer[index++] = response->slave_id;
        modbus->tx_buffer[index++] = response->function_code | 0x80;
        modbus->tx_buffer[index++] = response->exception_code;
    } else {
        // Normal response - data is already in tx_buffer, just add CRC
        index = 3 + response->data_len; // slave_id + function_code + byte_count + data
    }
    
    // Add CRC
    uint16_t crc = modbus_crc16(modbus->tx_buffer, index);
    modbus->tx_buffer[index++] = crc & 0xFF;        // Low byte first
    modbus->tx_buffer[index++] = (crc >> 8) & 0xFF; // High byte second
    
    // Send packet using callback
    if (modbus->callbacks.write_packet_callback) {
        modbus->callbacks.write_packet_callback(modbus->tx_buffer, index);
    }
}

// Modbus CRC16 calculation
uint16_t modbus_crc16(uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    uint8_t i;
    while (len--) {
        crc ^= *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

// Validate Modbus packet CRC
bool modbus_validate_crc(uint8_t *data, uint16_t data_len) {
    if (data_len < 2) return false;
    
    // In Modbus RTU, CRC is transmitted with low byte first, then high byte
    uint16_t received_crc = data[data_len - 2] | (data[data_len - 1] << 8);
    uint16_t calculated_crc = modbus_crc16(data, data_len - 2);
    
    return received_crc == calculated_crc;
}

// Validate holding register address range
static bool modbus_validate_holding_register_address(modbus_t *modbus, uint16_t start_address, uint16_t quantity) {
    // Check if start address is within allowed range
    if (start_address > modbus->max_holding_register_address) {
        return false;
    }
    
    // Check if the range (start_address + quantity - 1) is within allowed range
    // Use safe calculation to avoid overflow
    if (quantity == 0) {
        return false; // Quantity must be at least 1
    }
    
    uint16_t end_address = start_address + quantity - 1;
    if (end_address < start_address) {
        return false; // Overflow occurred
    }
    
    if (end_address > modbus->max_holding_register_address) {
        return false;
    }
    
    return true;
}

// Validate input register address range
static bool modbus_validate_input_register_address(modbus_t *modbus, uint16_t start_address, uint16_t quantity) {
    // Check if start address is within allowed range
    if (start_address > modbus->max_input_register_address) {
        return false;
    }
    
    // Check if the range (start_address + quantity - 1) is within allowed range
    // Use safe calculation to avoid overflow
    if (quantity == 0) {
        return false; // Quantity must be at least 1
    }
    
    uint16_t end_address = start_address + quantity - 1;
    if (end_address < start_address) {
        return false; // Overflow occurred
    }
    
    if (end_address > modbus->max_input_register_address) {
        return false;
    }
    
    return true;
}