/*
 * Example usage of multiple Modbus slaves with different serial drivers
 * This demonstrates how to set up multiple Modbus slaves on different UART ports
 */

#include "mb_slave_app.h"
#include "hal.h"

// Example: Set up multiple Modbus slaves on different UART ports
void setup_multiple_modbus_slaves(void) {
    // Initialize the Modbus application with the first slave on UART1
    mb_slave_app_init(1, &SD1);  // Slave ID 1 on UART1
    
    // Add additional slaves on different UART ports
    mb_slave_app_add_slave(2, &SD2);  // Slave ID 2 on UART2
    mb_slave_app_add_slave(3, &SD3);  // Slave ID 3 on UART3
    
    // Start the Modbus application
    mb_slave_app_start();
}

// Example: Get a specific slave instance
void example_register_access(void) {
    // Get slave with ID 2
    modbus_t *slave2 = mb_slave_app_get_slave(2);
    if (slave2) {
        // Access the slave's registers
        // This would typically be done through the callback functions
    }
    
    // Get the serial driver for slave ID 2
    SerialDriver *serial2 = mb_slave_app_get_serial(2);
    if (serial2) {
        // Access the serial driver if needed
    }
}

// Example: Remove a slave
void example_remove_slave(void) {
    // Remove slave with ID 3
    mb_slave_app_remove_slave(3);
}

// Example: Stop all slaves
void example_stop_all(void) {
    mb_slave_app_stop();
}


