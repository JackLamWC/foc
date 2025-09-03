# ChibiOS Naming and Coding Conventions for Custom Applications

This document outlines the naming and coding conventions for developing custom application code with ChibiOS, a real-time operating system (RTOS) for embedded systems. These conventions are derived from the ChibiOS source code (e.g., `hal_buffers.h`), its modular design, and embedded C programming best practices. Adhering to these guidelines ensures clarity, consistency, and seamless integration with ChibiOS.

## 1. General Principles
ChibiOS emphasizes:
- **Clarity**: Names should be descriptive and self-explanatory.
- **Consistency**: Uniform naming and structure across the codebase.
- **Modularity**: Code is organized into modules with clear separation of concerns.
- **C Compatibility**: Conventions align with C standards, using snake_case and avoiding C++-specific styles like camelCase or PascalCase.
- **Minimalism**: Avoid unnecessary complexity to suit resource-constrained embedded systems.

## 2. Header File Structure
Header files (`*.h`) define interfaces for modules and should follow a consistent structure, including the use of `/*===========================================================================*/` comment blocks to separate logical sections.

### 2.1 File Naming
- Use lowercase with underscores (snake_case).
- Reflect the module or functionality (e.g., `my_module.h`, `app_config.h`).
- Example: `sensor_driver.h` for a sensor module.

### 2.2 Include Guards
- Use include guards to prevent multiple inclusions.
- Format: `#ifndef PREFIX_MODULE_H`, `#define PREFIX_MODULE_H`, `#endif`.
- Use a unique project prefix (e.g., `MYAPP_`) to avoid conflicts with ChibiOS (`CH_` or `HAL_`).
- Example:
  ```c
  #ifndef MYAPP_SENSOR_H
  #define MYAPP_SENSOR_H
  // Header content
  #endif /* MYAPP_SENSOR_H */
  ```

### 2.3 License/Copyright Notice
- Include a license header at the top, matching ChibiOS’s style (e.g., Apache License 2.0 or MIT License).
- Example:
  ```c
  /*
   * Copyright (c) 2025 Your Name/Organization
   * Licensed under the terms of the MIT License.
   */
  ```

### 2.4 Section Organization with Comment Blocks
- Use `/*===========================================================================*/` to separate logical sections, as seen in ChibiOS files like `hal_buffers.h`.
- Each section starts with `/*===========================================================================*/`, followed by a descriptive comment (e.g., `/* Driver constants. */`), and ends with another `/*===========================================================================*/`.
- Common sections in headers:
  - **Driver constants**: Fixed values used by the module.
  - **Driver pre-compile time settings**: User-configurable macros.
  - **Derived constants and error checks**: Computed values or validation checks.
  - **Driver data structures and types**: Structs, typedefs, and enums.
  - **Driver macros**: Utility macros or inline functions.
  - **External declarations**: Function prototypes or external variables.
- Example:
  ```c
  /*===========================================================================*/
  /* Driver constants.                                                         */
  /*===========================================================================*/
  #define MYAPP_MAX_BUFFER_SIZE 128
  ```

### 2.5 Includes
- List ChibiOS headers first (e.g., `ch.h`, `hal.h`), followed by standard C headers, then custom headers.
- Use forward slashes for portability.
- Include only necessary headers to minimize dependencies.
- Place in a section labeled `/* External dependencies. */` if following ChibiOS style.
- Example:
  ```c
  /*===========================================================================*/
  /* External dependencies.                                                    */
  /*===========================================================================*/
  #include "ch.h"
  #include "hal.h"
  #include "my_config.h"
  ```

### 2.6 Macros and Defines
- Use uppercase snake_case with a project prefix.
- Reflect the purpose clearly.
- Use ChibiOS macros like `MS2ST` for time conversions.
- Place in `/* Driver constants. */` or `/* Driver pre-compile time settings. */` sections.
- Example:
  ```c
  /*===========================================================================*/
  /* Driver constants.                                                         */
  /*===========================================================================*/
  #define MYAPP_MAX_BUFFER_SIZE 128
  #define MYAPP_DEFAULT_TIMEOUT MS2ST(100) /* 100ms in system ticks */
  ```

### 2.7 Type Definitions
- Use `typedef` for structs and enums with a project prefix.
- Include a struct tag for clarity.
- Place in `/* Driver data structures and types. */` section.
- Example:
  ```c
  /*===========================================================================*/
  /* Driver data structures and types.                                         */
  /*===========================================================================*/
  typedef struct {
    uint8_t id;
    uint16_t value;
  } myapp_config_t;
  ```

### 2.8 Function Prototypes
- Use snake_case with a project prefix (e.g., `myapp_`).
- Use `msg_t` for functions returning ChibiOS status codes (e.g., `MSG_OK`).
- Place in `/* External declarations. */` section.
- Example:
  ```c
  /*===========================================================================*/
  /* External declarations.                                                    */
  /*===========================================================================*/
  void myapp_init(void);
  msg_t myapp_process_data(uint8_t *data, size_t len);
  ```

### 2.9 Documentation
- Use Doxygen-compatible comments for the file, types, and functions.
- Include `@file`, `@brief`, `@param`, `@return`, and `@details` as needed.
- Place file-level documentation before the include guard.
- Example:
  ```c
  /**
   * @file    my_module.h
   * @brief   Custom module for handling sensor data.
   */
  ```

### 2.10 Example Header File
```c
/*
 * Copyright (c) 2025 Your Name
 * Licensed under the terms of the MIT License.
 */

/**
 * @file    myapp_sensor.h
 * @brief   Custom sensor module for ChibiOS.
 */

#ifndef MYAPP_SENSOR_H
#define MYAPP_SENSOR_H

/*===========================================================================*/
/* External dependencies.                                                    */
/*===========================================================================*/
#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/
#define MYAPP_SENSOR_MAX_READINGS 10
#define MYAPP_SENSOR_TIMEOUT MS2ST(50)

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
typedef struct {
  uint16_t readings[MYAPP_SENSOR_MAX_READINGS];
  uint8_t count;
} myapp_sensor_data_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void myapp_sensor_init(void);
msg_t myapp_sensor_read(myapp_sensor_data_t *data);

#endif /* MYAPP_SENSOR_H */
```

## 3. Source File Structure
Source files (`*.c`) implement the functionality declared in headers and should use `/*===========================================================================*/` blocks to organize content.

### 3.1 File Naming
- Match the corresponding header (e.g., `my_module.c` for `my_module.h`).

### 3.2 Includes
- Include the corresponding header first, followed by other dependencies.
- Use a `/* External dependencies. */` section.
- Example:
  ```c
  /*===========================================================================*/
  /* External dependencies.                                                    */
  /*===========================================================================*/
  #include "myapp_sensor.h"
  #include "ch.h"
  #include "hal.h"
  ```

### 3.3 Static Variables and Functions
- Use `static` for module-internal variables and functions to limit scope.
- Use the project prefix for clarity.
- Place in `/* Driver local variables. */` or `/* Driver local functions. */` sections.
- Example:
  ```c
  /*===========================================================================*/
  /* Driver local variables.                                                   */
  /*===========================================================================*/
  static uint8_t myapp_internal_buffer[MYAPP_MAX_BUFFER_SIZE];
  ```

### 3.4 Function Implementation
- Implement functions declared in the header.
- Use Doxygen-style comments with `@brief`, `@param`, and `@return`.
- Place in `/* Driver exported functions. */` section for public functions or `/* Driver local functions. */` for static functions.
- Example:
  ```c
  /*===========================================================================*/
  /* Driver exported functions.                                                */
  /*===========================================================================*/
  /**
   * @brief   Initializes the sensor module.
   * @details Configures the sensor hardware and resets internal state.
   */
  void myapp_sensor_init(void) {
    chSysLock();
    // Critical section
    chSysUnlock();
  }
  ```

### 3.5 Thread Definitions
- Use `THD_FUNCTION` for thread functions with a project prefix.
- Declare thread working areas with a descriptive name.
- Place in `/* Driver local functions. */` or a dedicated `/* Driver threads. */` section.
- Example:
  ```c
  /*===========================================================================*/
  /* Driver local variables.                                                   */
  /*===========================================================================*/
  static THD_WORKING_AREA(myapp_sensor_wa, 128);

  /*===========================================================================*/
  /* Driver threads.                                                           */
  /*===========================================================================*/
  THD_FUNCTION(myapp_sensor_thread, arg) {
    (void)arg;
    chRegSetThreadName("Sensor Thread");
    while (!chThdShouldTerminateX()) {
      // Thread logic
      chThdSleepMilliseconds(100);
    }
  }
  ```

### 3.6 Example Source File
```c
/*
 * Copyright (c) 2025 Your Name
 * Licensed under the terms of the MIT License.
 */

/**
 * @file    myapp_led.c
 * @brief   LED control module for ChibiOS.
 */

#include "myapp_led.h"

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/
static bool is_led_initialized = false;

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/
/**
 * @brief   Initializes the LED module.
 */
void myapp_led_init(void) {
  palSetPadMode(GPIOD, GPIOD_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
  is_led_initialized = true;
}

/**
 * @brief   Toggles the LED state.
 */
void myapp_led_toggle(void) {
  if (is_led_initialized) {
    palTogglePad(GPIOD, GPIOD_LED_GREEN);
  }
}

/**
 * @brief   Sets the LED state.
 * @param[in] state  True to turn on, false to turn off.
 */
void myapp_led_set(bool state) {
  if (is_led_initialized) {
    if (state) {
      palSetPad(GPIOD, GPIOD_LED_GREEN);
    } else {
      palClearPad(GPIOD, GPIOD_LED_GREEN);
    }
  }
}

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/
static THD_WORKING_AREA(myapp_led_wa, 128);

/*===========================================================================*/
/* Driver threads.                                                           */
/*===========================================================================*/
THD_FUNCTION(myapp_led_thread, arg) {
  (void)arg;
  chRegSetThreadName("LED Blinker");
  while (!chThdShouldTerminateX()) {
    myapp_led_toggle();
    chThdSleepMilliseconds(MYAPP_LED_BLINK_INTERVAL);
  }
}
```

## 4. Variable Naming Conventions
Variables should be descriptive and follow a consistent style.

- **Format**: Use snake_case (lowercase with underscores).
- **Descriptive Names**: Reflect the variable’s purpose (e.g., `sensor_value` instead of `val`).
- **Project Prefix**: Use a prefix (e.g., `myapp_`) for global or static variables.
  ```c
  static uint16_t myapp_adc_reading;
  static bool myapp_is_initialized;
  ```
- **Boolean Variables**: Use prefixes like `is_`, `has_`, or `should_` to imply true/false.
  ```c
  bool is_sensor_active;
  bool has_data_ready;
  ```
- **Constants**: Use uppercase snake_case with a project prefix.
  ```c
  #define MYAPP_MAX_RETRIES 5
  static const uint8_t MYAPP_DEFAULT_ADDRESS = 0x10;
  ```
- **Avoid Hungarian Notation**: Do not prefix types (e.g., `iCount`). Use Apps Hungarian sparingly for purpose (e.g., `raw_adc_value`).
- **ChibiOS Objects**: Use descriptive names for threads, semaphores, and mutexes.
  ```c
  static semaphore_t myapp_data_sem;
  static mutex_t myapp_config_mutex;
  ```

## 5. Function Naming Conventions
Functions should clearly indicate their action and scope.

- **Format**: Use snake_case with a project prefix.
- **Verb-Noun Structure**: Start with a verb (e.g., `init`, `read`, `process`) followed by a noun.
  ```c
  void myapp_led_toggle(void);
  msg_t myapp_sensor_read(uint8_t *buffer);
  ```
- **Return Types**: Use `msg_t` for ChibiOS status codes, `void` for procedures.
- **Static Functions**: Use `static` for internal functions, maintaining the same naming style.
  ```c
  static void myapp_internal_calculate(void);
  ```

## 6. Code Organization and Best Practices
- **Modularity**: Group code into modules (e.g., `sensor`, `motor`) with separate header/source files.
- **Consistent Prefix**: Use a project-specific prefix (e.g., `myapp_`) for all identifiers.
- **Comment Blocks**: Use `/*===========================================================================*/` to separate sections, as seen in `hal_buffers.h`. Common sections include:
  - `Driver constants`
  - `Driver pre-compile time settings`
  - `Derived constants and error checks`
  - `Driver data structures and types`
  - `Driver macros`
  - `External declarations`
  - `Driver local variables`
  - `Driver local functions`
  - `Driver exported functions`
  - `Driver threads`
- **Coding Style**:
  - Use 2-space indentation (no tabs).
  - Place braces on the same line:
    ```c
    if (condition) {
      // Code
    }
    ```
  - Keep lines under 80–100 characters.
- **ChibiOS Idioms**: Use macros like `chSysLock()`, `chThdSleepMilliseconds()`, and `MS2ST`.
- **Documentation**: Use Doxygen comments for all public interfaces.
  ```c
  /**
   * @brief   Reads sensor data.
   * @param[out] data   Output buffer.
   * @param[in]  len    Buffer length.
   * @return            Operation status.
   * @retval MSG_OK     Success.
   * @retval MSG_TIMEOUT Timeout occurred.
   */
  msg_t myapp_sensor_read(uint8_t *data, size_t len);
  ```

## 7. Example Application
Below is an example of a complete module for controlling an LED, using `/*===========================================================================*/` blocks.

### 7.1 Header: `myapp_led.h`
```c
/*
 * Copyright (c) 2025 Your Name
 * Licensed under the terms of the MIT License.
 */

/**
 * @file    myapp_led.h
 * @brief   LED control module for ChibiOS.
 */

#ifndef MYAPP_LED_H
#define MYAPP_LED_H

/*===========================================================================*/
/* External dependencies.                                                    */
/*===========================================================================*/
#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/
#define MYAPP_LED_BLINK_INTERVAL MS2ST(500)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void myapp_led_init(void);
void myapp_led_toggle(void);
void myapp_led_set(bool state);

#endif /* MYAPP_LED_H */
```

### 7.2 Source: `myapp_led.c`
```c
/*
 * Copyright (c) 2025 Your Name
 * Licensed under the terms of the MIT License.
 */

/**
 * @file    myapp_led.c
 * @brief   LED control module for ChibiOS.
 */

#include "myapp_led.h"

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/
static bool is_led_initialized = false;

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/
/**
 * @brief   Initializes the LED module.
 */
void myapp_led_init(void) {
  palSetPadMode(GPIOD, GPIOD_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
  is_led_initialized = true;
}

/**
 * @brief   Toggles the LED state.
 */
void myapp_led_toggle(void) {
  if (is_led_initialized) {
    palTogglePad(GPIOD, GPIOD_LED_GREEN);
  }
}

/**
 * @brief   Sets the LED state.
 * @param[in] state  True to turn on, false to turn off.
 */
void myapp_led_set(bool state) {
  if (is_led_initialized) {
    if (state) {
      palSetPad(GPIOD, GPIOD_LED_GREEN);
    } else {
      palClearPad(GPIOD, GPIOD_LED_GREEN);
    }
  }
}

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/
static THD_WORKING_AREA(myapp_led_wa, 128);

/*===========================================================================*/
/* Driver threads.                                                           */
/*===========================================================================*/
THD_FUNCTION(myapp_led_thread, arg) {
  (void)arg;
  chRegSetThreadName("LED Blinker");
  while (!chThdShouldTerminateX()) {
    myapp_led_toggle();
    chThdSleepMilliseconds(MYAPP_LED_BLINK_INTERVAL);
  }
}
```

### 7.3 Main: `main.c`
```c
/*
 * Copyright (c) 2025 Your Name
 * Licensed under the terms of the MIT License.
 */

/**
 * @file    main.c
 * @brief   Main application for ChibiOS.
 */

#include "ch.h"
#include "hal.h"
#include "myapp_led.h"

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/
int main(void) {
  halInit();
  chSysInit();

  myapp_led_init();
  chThdCreateStatic(myapp_led_wa, sizeof(myapp_led_wa), NORMALPRIO, myapp_led_thread, NULL);

  while (true) {
    chThdSleepMilliseconds(1000);
  }
}
```

## 8. Additional Tips
- **Study ChibiOS Source Code**: Analyze files like `hal_buffers.h` and demos in the ChibiOS repository (e.g., `demos/`) for practical examples.
- **Use Tools**: Use `clang-format` with a ChibiOS-compatible style file for consistent formatting.
- **Check Community Resources**: Refer to the ChibiOS forum or GitHub for community practices.
- **Avoid Name Conflicts**: Do not use `ch_` or `hal_` prefixes, as they are reserved for ChibiOS.
- **Reference Manual**: The ChibiOS reference manual (available at [ChibiOS Documentation](http://www.chibios.org)) focuses on API usage but does not explicitly detail coding style. Emulate the source code for conventions like `/*===========================================================================*/` blocks.

## 9. References
- ChibiOS source code (e.g., `hal_buffers.h`) and demos (available on GitHub).
- ChibiOS reference manual (http://www.chibios.org).
- General C conventions (e.g., Google C++ Style Guide adapted for C).
- Doxygen documentation guidelines for embedded systems.

By following these conventions, including the use of `/*===========================================================================*/` comment blocks, your custom application will align with ChibiOS’s style, ensuring maintainability and integration with the RTOS.