#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_MCP4725.h"

#include "settings.h"

Adafruit_MCP4725 dac;

uint16_t currentOutput = 0;

/**
 * @brief Linearly map a 0-255 UART byte to the DAC range 0-4095.
 *
 * @param input Duty cycle from 0-255
 * @return Voltage value from 0-4095
 * */
uint16_t mapUartToDac(uint8_t input) {
    return (uint32_t)input * 4095U / 255U;
}

/**
 *  SETUP
 * */
void setup() {
    // Start Serial for raw UART input
    Serial.begin(UART_RAW_BAUD);

    DEBUG_SERIAL_LN();
    DEBUG_SERIAL_LN("*************************");
    DEBUG_SERIAL_LN("    UART THROTTLEATOR    ");
    DEBUG_SERIAL_LN("*************************"); 
    DEBUG_SERIAL_LN();
    
    // Start DAC
    while(!dac.begin(I2C_ADDR_DAC)) {
        DEBUG_SERIAL_LN("DAC INIT: DAC ERROR");
        delay(250);
    }
    DEBUG_SERIAL_LN("DAC INIT: DAC OK");

    // Set DAC to 0 and save it in memory
    dac.setVoltage(0, true);
}

/**
 *  LOOP
 * */
void loop() {

    uint16_t newOutput = currentOutput;

    while (Serial.available() > 0) {
        uint8_t duty = (uint8_t)Serial.read();
        newOutput = mapUartToDac(duty);
    }

    // If the new output is different than the old output, update the voltage on the DAC
    if(newOutput != currentOutput) {
        currentOutput = newOutput;

        dac.setVoltage(currentOutput, false); 
        if (DEBUG_SERIAL_EN) {
            DEBUG_SERIAL_LN("DAC set to: " + String(currentOutput));
        }
    }
}
