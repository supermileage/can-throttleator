#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_MCP4725.h"

#include "settings.h"

Adafruit_MCP4725 dac;

uint16_t currentOutput = 0;
static const int kConfirmThreshold = 10;

static void applyDuty(int duty) {

    currentOutput = (uint16_t)((4095UL * duty) / 100UL);
    dac.setVoltage(currentOutput, false);
    DEBUG_SERIAL_LN("Dutycycle set to: " + String(duty) + "%");
    float volts = (float)currentOutput / (float)DAC_VALUE_TO_V;
    DEBUG_SERIAL_LN("Voltage set to  : " + String(volts, 2) + "v");
    DEBUG_SERIAL_LN();
}

static bool parseDuty(const char* buf, uint8_t len, int* outDuty) {
    int duty = 0;
    for (uint8_t i = 0; i < len; i++) {
        if (!isDigit(buf[i])) {
            return false;
        }
        duty = (duty * 10) + (buf[i] - '0');
    }
    *outDuty = duty;
    return len > 0;
}

/**
 *  SETUP
 * */
void setup() {
    // Start Serial for raw UART input
    Serial.begin(UART_RAW_BAUD);

    DEBUG_SERIAL_LN();
    DEBUG_SERIAL_LN("*************************");
    DEBUG_SERIAL_LN("    TEST THROTTLEATOR    ");
    DEBUG_SERIAL_LN("*************************"); 
    DEBUG_SERIAL_LN();
    
    // Start DAC
    while(!dac.begin(I2C_ADDR_DAC)) {
        DEBUG_SERIAL_LN("DAC INIT: DAC ERROR");
        delay(250);
    }
    DEBUG_SERIAL_LN("DAC INIT: DAC OK");
    DEBUG_SERIAL_LN();
    DEBUG_SERIAL_LN("Enter desired dutycycle percent (0-100):");
    DEBUG_SERIAL_LN();

    // Set DAC to 0 and save it in memory
    dac.setVoltage(0, true);
}

/**
 *  LOOP
 * */
void loop() {

    static char rxBuf[4];
    static uint8_t rxLen = 0;
    static bool pendingHighConfirm = false;
    static int pendingHighDuty = -1;

    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\b' || c == 127) {
            if (rxLen > 0) {
                rxLen--;
                Serial.write('\b');
                Serial.write(' ');
                Serial.write('\b');
            }
            continue;
        }

        Serial.write(c); // echo each character as it arrives

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            if (rxLen == 0) {
                continue;
            }
            int duty = 0;
            if (!parseDuty(rxBuf, rxLen, &duty)) {
                pendingHighConfirm = false;
                pendingHighDuty = -1;
                DEBUG_SERIAL_LN("ERROR: dutycycle must be a number 0-100. Remove non-numeric characters and try again.\n");
            } else if (duty > 100) {
                pendingHighConfirm = false;
                pendingHighDuty = -1;
                DEBUG_SERIAL_LN("ERROR: dutycycle must be 0-100. Send a valid value and try again.\n");
            } else if (duty > kConfirmThreshold - 1 && (!pendingHighConfirm || pendingHighDuty != duty)) {
                pendingHighConfirm = true;
                pendingHighDuty = duty;
                DEBUG_SERIAL_LN("CONFIRM: dutycycle > 10. Send the same value again to apply.");
            } else {
                pendingHighConfirm = false;
                pendingHighDuty = -1;
                applyDuty(duty);
            }
            rxLen = 0;
        } else {
            if (rxLen < (sizeof(rxBuf) - 1)) {
                rxBuf[rxLen++] = c;
            } else {
                pendingHighConfirm = false;
                pendingHighDuty = -1;
                DEBUG_SERIAL_LN();
                DEBUG_SERIAL_LN("ERROR: input too long. Max 3 characters. Send a shorter value.");
                rxLen = 0;
            }
        }
    }
}
