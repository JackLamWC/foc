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
    uint8_t *data;
    uint16_t data_len;
} modbus_request_t;

typedef struct {
    uint8_t slave_id;
    uint8_t function_code;
    uint8_t exception_code;
    uint8_t *data;
    uint16_t data_len;
} modbus_response_t;


// Private Functions
static void modbus_send_response(modbus_t *modbus, modbus_response_t *response);

// Modbus Register Read Functions - these will use callbacks if registered
static uint16_t modbus_read_holding_register(modbus_t *modbus, uint16_t address);
static void modbus_write_holding_register(modbus_t *modbus, uint16_t address, uint16_t value);
static uint16_t modbus_read_input_register(modbus_t *modbus, uint16_t address);

void modbus_init(modbus_t *modbus, uint8_t slave_id, 
                 modbus_write_callback_t write_packet_callback,
                 modbus_holding_register_read_callback_t holding_register_read_callback,
                 modbus_holding_register_write_callback_t holding_register_write_callback,
                 modbus_input_register_read_callback_t input_register_read_callback) {
    modbus->slave_id = slave_id;
    modbus->write_packet_callback = write_packet_callback;
    modbus->holding_register_read_callback = holding_register_read_callback;
    modbus->holding_register_write_callback = holding_register_write_callback;
    modbus->input_register_read_callback = input_register_read_callback;
    
    // Initialize instance-specific buffers
    memset(modbus->rx_buffer, 0, sizeof(modbus->rx_buffer));
    memset(modbus->tx_buffer, 0, sizeof(modbus->tx_buffer));
}

void modbus_clear_buffers(modbus_t *modbus) {
    if (modbus) {
        memset(modbus->rx_buffer, 0, sizeof(modbus->rx_buffer));
        memset(modbus->tx_buffer, 0, sizeof(modbus->tx_buffer));
    }
}

// Process Modbus request packets
modbus_process_error_t modbus_process_packet(modbus_t *modbus, uint8_t *data, uint16_t len) {
    if (len < 4) return MODBUS_PROCESS_ERROR_INVALID_LENGTH; // Minimum packet size
    
    // Validate CRC
    if (!modbus_validate_crc(data, len)) {
        return MODBUS_PROCESS_ERROR_INVALID_CRC; // Invalid CRC, ignore packet
    }
    
    modbus_request_t request;
    modbus_response_t response;
    
    // Parse Modbus request
    request.slave_id = data[0];
    request.function_code = data[1];
    request.start_address = (data[2] << 8) | data[3];
    
    // Check if this request is for us
    if (request.slave_id != modbus->slave_id && request.slave_id != 0) {
        return MODBUS_PROCESS_ERROR_WRONG_SLAVE_ID; // Not for us
    }
    
    // Initialize response
    response.slave_id = modbus->slave_id;
    response.function_code = request.function_code;
    response.exception_code = MODBUS_EX_NONE;
    response.data = modbus->tx_buffer + 3; // Skip slave_id, function_code, and byte_count
    response.data_len = 0;
    
    // Process based on function code
    switch (request.function_code) {
        case MODBUS_FC_READ_HOLDING_REG: {
            if (len >= 6) {
                request.quantity = (data[4] << 8) | data[5];
                
                if (request.quantity > 0 && request.quantity <= 125) {
                    response.data_len = request.quantity * 2;
                    
                    // Check buffer overflow
                    if (response.data_len + 3 > MODBUS_BUFFER_SIZE) {
                        return MODBUS_PROCESS_ERROR_BUFFER_OVERFLOW;
                    }
                    
                    response.data[0] = response.data_len; // Byte count
                    
                    // Read holding registers using callback
                    for (int i = 0; i < request.quantity; i++) {
                        uint16_t reg_value = modbus_read_holding_register(modbus, request.start_address + i);
                        response.data[1 + i * 2] = (reg_value >> 8) & 0xFF;
                        response.data[2 + i * 2] = reg_value & 0xFF;
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
                
                if (request.quantity > 0 && request.quantity <= 125) {
                    response.data_len = request.quantity * 2;
                    
                    // Check buffer overflow
                    if (response.data_len + 3 > MODBUS_BUFFER_SIZE) {
                        return MODBUS_PROCESS_ERROR_BUFFER_OVERFLOW;
                    }
                    
                    response.data[0] = response.data_len; // Byte count
                    
                    // Read input registers using callback
                    for (int i = 0; i < request.quantity; i++) {
                        uint16_t reg_value = modbus_read_input_register(modbus, request.start_address + i);
                        response.data[1 + i * 2] = (reg_value >> 8) & 0xFF;
                        response.data[2 + i * 2] = reg_value & 0xFF;
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
                modbus_write_holding_register(modbus, request.start_address, reg_value);
                
                // Echo back the write request
                response.data_len = 4;
                
                // Check buffer overflow
                if (response.data_len + 3 > MODBUS_BUFFER_SIZE) {
                    return MODBUS_PROCESS_ERROR_BUFFER_OVERFLOW;
                }
                
                response.data[0] = (request.start_address >> 8) & 0xFF;
                response.data[1] = request.start_address & 0xFF;
                response.data[2] = (reg_value >> 8) & 0xFF;
                response.data[3] = reg_value & 0xFF;
            } else {
                response.exception_code = MODBUS_EX_ILLEGAL_DATA_VAL;
            }
            break;
        }
        
        case MODBUS_FC_WRITE_MULTIPLE_REG: {
            if (len >= 7) {
                request.quantity = (data[4] << 8) | data[5];
                uint8_t byte_count = data[6];
                
                if (request.quantity > 0 && request.quantity <= 123 && 
                    len >= 7 + byte_count && byte_count == request.quantity * 2) {
                    
                    // Write holding registers using callback
                    for (int i = 0; i < request.quantity; i++) {
                        uint16_t reg_value = (data[7 + i * 2] << 8) | data[8 + i * 2];
                        modbus_write_holding_register(modbus, request.start_address + i, reg_value);
                    }
                    
                    // Echo back the write request
                    response.data_len = 4;
                    
                    // Check buffer overflow
                    if (response.data_len + 3 > MODBUS_BUFFER_SIZE) {
                        return MODBUS_PROCESS_ERROR_BUFFER_OVERFLOW;
                    }
                    
                    response.data[0] = (request.start_address >> 8) & 0xFF;
                    response.data[1] = request.start_address & 0xFF;
                    response.data[2] = (request.quantity >> 8) & 0xFF;
                    response.data[3] = request.quantity & 0xFF;
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

// Read Modbus packet with callback support
uint8_t modbus_read_packet(uint8_t *buffer, uint16_t index, uint16_t length) {
    // This function can be used to read packet data with custom logic
    // The actual implementation depends on the callback registered
    if (buffer && length > 0) {
        // Basic packet reading logic
        return 1; // Success
    }
    return 0; // Failure
}

// Send Modbus response
static void modbus_send_response(modbus_t *modbus, modbus_response_t *response) {
    uint16_t index = 0;
    
    // Build response packet
    modbus->tx_buffer[index++] = response->slave_id;
    
    if (response->exception_code != MODBUS_EX_NONE) {
        // Exception response
        modbus->tx_buffer[index++] = response->function_code | 0x80;
        modbus->tx_buffer[index++] = response->exception_code;
    } else {
        // Normal response
        modbus->tx_buffer[index++] = response->function_code;
        
        // Copy data
        if (response->data_len > 0) {
            memcpy(&modbus->tx_buffer[index], response->data, response->data_len);
            index += response->data_len;
        }
    }
    
    // Add CRC
    uint16_t crc = modbus_crc16(modbus->tx_buffer, index);
    modbus->tx_buffer[index++] = crc & 0xFF;
    modbus->tx_buffer[index++] = (crc >> 8) & 0xFF;
    
    // Send packet using callback
    if (modbus->write_packet_callback) {
        modbus->write_packet_callback(modbus->tx_buffer, index);
    }
}

// Modbus Register Read Functions using callbacks
static uint16_t modbus_read_holding_register(modbus_t *modbus, uint16_t address) {
    if (modbus->holding_register_read_callback) {
        return modbus->holding_register_read_callback(address);
    }
    return 0; // Default value if no callback registered
}

static void modbus_write_holding_register(modbus_t *modbus, uint16_t address, uint16_t value) {
    if (modbus->holding_register_write_callback) {
        modbus->holding_register_write_callback(address, value);
    }
}

static uint16_t modbus_read_input_register(modbus_t *modbus, uint16_t address) {
    if (modbus->input_register_read_callback) {
        return modbus->input_register_read_callback(address);
    }
    return 0; // Default value if no callback registered
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
    return crc << 8 | crc >> 8;
}

// Validate Modbus packet CRC
bool modbus_validate_crc(uint8_t *data, uint16_t data_len) {
    if (data_len < 2) return false;
    
    // In Modbus RTU, CRC is transmitted with low byte first, then high byte
    uint16_t received_crc = (data[data_len - 2] << 8) | data[data_len - 1];
    uint16_t calculated_crc = modbus_crc16(data, data_len - 2);
    
    return received_crc == calculated_crc;
}