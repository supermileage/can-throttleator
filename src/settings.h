#pragma once

#include "mcp2515_can.h"

// Turn on to enable serial monitor
#define DEBUG_SERIAL_EN         1 // Turn on
#define SERIAL_MONITOR_SPEED    115200

// Hall Sensor Pin for RPM Measurement
#define INTERRUPT_A 2 // Connected to PIN_HALL_A
#define RPM_LIMITER 0 // Enables throttle limiting based on RPM Measurement
#define MIN_THROTTLE 1000
#define TIME_INTERVAL 50

// After this time in ms of no throttle CAN messages, throttle set to 0
#define STALE_TIME              1000
// Heartbeat CAN message output interval
#define HEARTBEAT_TIME          1000

#define PIN_CAN_CS              10
#define PIN_LED                 5

#define I2C_ADDR_DAC            0x60

#define CAN_SPEED               CAN_500KBPS
#define CAN_CONTROLLER_SPEED    MCP_8MHz
#define CAN_FRAME               0x0

// Turn on to print received CAN messages to serial monitor
#define CAN_DEBUG_RECEIVE       0 // Print to serial monitor every CAN message (turn on)

// Turn on to scale exponentially
#define SCALE_EXP               1

// Rolling Average
#define DATA_POINTS             20
// Scaling factors
#define DAC_VALUE_TO_V          819.2
#define DAC_VALUE_TO_INPUT      16U

#define LED_FLASH_INTERVAL      125

// Serial monitor macros
#if DEBUG_SERIAL_EN
    #define DEBUG_SERIAL_LN(x) Serial.println(x)
    #define DEBUG_SERIAL(x) Serial.print(x)
#else
    #define DEBUG_SERIAL_LN(x) x
    #define DEBUG_SERIAL(x) x
#endif