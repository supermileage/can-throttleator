#pragma once

// Turn on to enable serial debug output.
// Note: debug output shares the same UART used for input bytes.
#define DEBUG_SERIAL_EN         1

// UART settings for raw byte input (0-255)
#define UART_RAW_BAUD           115200

// DAC I2C address
#define I2C_ADDR_DAC            0x62

// Serial monitor macros
#if DEBUG_SERIAL_EN
    #define DEBUG_SERIAL_LN(x) Serial.println(x)
    #define DEBUG_SERIAL(x) Serial.print(x)
#else
    #define DEBUG_SERIAL_LN(x) x
    #define DEBUG_SERIAL(x) x
#endif
