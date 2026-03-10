#pragma once

#include "mcp2515_can.h"

// Turn on to enable serial debug output.
// Note: debug output shares the same UART used for input bytes.
#define DEBUG_SERIAL_EN         1

// UART settings for raw byte input (0-255)
#define UART_RAW_BAUD           115200

#define PIN_CAN_CS              10
#define PIN_HALL_IT             2

// DAC I2C address
#define I2C_ADDR_DAC            0x62

#define CAN_SPEED               CAN_500KBPS
#define CAN_CONTROLLER_SPEED    MCP_8MHz
#define CAN_FRAME               0x0

// Turn on to print received CAN messages to serial monitor
#define CAN_DEBUG_RECEIVE       0 // Print to serial monitor every CAN message (turn on)

// Scaling factors
#define DAC_VALUE_TO_V          819.2
#define DAC_VALUE_TO_INPUT      16U

// Serial monitor macros
#if DEBUG_SERIAL_EN
    #define DEBUG_SERIAL_LN(x) Serial.println(x)
    #define DEBUG_SERIAL(x) Serial.print(x)
#else
    #define DEBUG_SERIAL_LN(x) x
    #define DEBUG_SERIAL(x) x
#endif
