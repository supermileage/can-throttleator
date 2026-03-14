#pragma once

#include "mcp2515_can.h"

// Turn on to enable serial monitor
#define DEBUG_SERIAL_EN         0
#define SERIAL_MONITOR_SPEED    115200

// After this time in ms of no throttle CAN messages, throttle set to 0
#define STALE_TIME              1000
// Heartbeat CAN message output interval
#define HEARTBEAT_TIME          1000

// DAC
#define DAC_VALUE_TO_V          819.2
#define DAC_VALUE_TO_INPUT      16U
#define DAC_3_3V                2703
#define DAC_MAX_CMD             2433  // 90 percent

// Current liming constants
#define CURRENT_FILTER_ALPHA    0.15f
#define SOFT_CURRENT_LIMIT_A    40.0f  // start reducing throttle above this
#define HARD_CURRENT_LIMIT_A    60.0f  // force throttle to 0 above this
#define CURRENT_TIMEOUT_MS      1000   // if BMS CAN message is stale, throttle 0
#define OUTPUT_RAMP_UP_PCT_PER_LOOP 0.01f  // every loop, go up 1 percent towards requested throttle

#define PIN_CAN_CS              10
#define PIN_LED                 5
#define PIN_HEARTBEAT           0

// #define I2C_ADDR_DAC            0x60  // PROTO DAC
#define I2C_ADDR_DAC            0x62  // PCB DAC

#define CAN_SPEED               CAN_500KBPS
#define CAN_CONTROLLER_SPEED    MCP_8MHz
#define CAN_FRAME               0x0

// Turn on to print received CAN messages to serial monitor
#define CAN_DEBUG_RECEIVE       0 // Print to serial monitor every CAN message (turn on)

// Turn on to implement a rolling average on throttle input, mitigating spikes and noise
#define SMOOTH_THROTTLE         1

// Rolling Average
#define DATA_POINTS             20

#define LED_FLASH_INTERVAL      125

// Serial monitor macros
#if DEBUG_SERIAL_EN
    #define DEBUG_SERIAL_LN(x) Serial.println(x)
    #define DEBUG_SERIAL(x) Serial.print(x)
#else
    #define DEBUG_SERIAL_LN(x) x
    #define DEBUG_SERIAL(x) x
#endif
