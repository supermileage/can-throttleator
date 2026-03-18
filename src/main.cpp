#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_MCP4725.h"
#include "can_common.h"

#include "settings.h"

mcp2515_can can(PIN_CAN_CS);
Adafruit_MCP4725 dac;

uint16_t currentOutput = 0;
static const int kConfirmThreshold = 100;

// This struct contains all the components of a CAN message. dataLength must be <= 8, 
// and the first [dataLength] positions of data[] must contain valid data
typedef uint8_t CanBuffer[8];
struct CanMessage {
        uint32_t id;
        uint8_t dataLength;
        CanBuffer data;
};

uint32_t lastUpdate = 0;
float rpm = 0.0f;
float filteredRpm = 0.0f;
float packCurrent = 0.0f;
float filteredCurrent = 0.0f;
bool hasCurrentReading = false;
volatile uint32_t pulses = 0;

static const uint16_t kTelemetryIntervalMs = 200;
static const float kRpmFilterAlpha = 0.25f;
static const float kCurrentFilterAlpha = 0.20f;

/**
 * @brief Converts CAN status message to readable output
 * 
 * @param errorCode CAN status message
 * @return Readable output
 * */
String getCanError(uint8_t errorCode){
    switch(errorCode){
        case CAN_OK: 
            return "CAN OK";
            break;
        case CAN_FAILINIT:
            return "CAN FAIL INIT";
            break;
        case CAN_FAILTX:
            return "CAN FAIL TX";
            break;
        case CAN_MSGAVAIL:
            return "CAN MSG AVAIL";
            break;
        case CAN_NOMSG:
            return "CAN NO MSG";
            break;
        case CAN_CTRLERROR:
            return "CAN CTRL ERROR";
            break;
        case CAN_GETTXBFTIMEOUT:
            return "CAN TX BF TIMEOUT";
            break;
        case CAN_SENDMSGTIMEOUT:    
            return "CAN SEND MSG TIMEOUT";
            break;
        default:
            return "CAN FAIL";
            break;
    }
}

static void applyDuty(int duty) {

    currentOutput = (uint16_t)((4095UL * duty) / 100UL) * 0.67; // cap at 67% cuz stm takes in 3.3 not 5v silly goose
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

static float lowPassFilter(float previous, float sample, float alpha) {
    return previous + ((sample - previous) * alpha);
}

static void printTelemetry() {
    Serial.print("RPM ");
    Serial.print(filteredRpm, 0);
    Serial.print(" | Current ");
    if (hasCurrentReading) {
        Serial.print(filteredCurrent, 1);
        Serial.println(" A");
    } else {
        Serial.println("--.- A");
    }
}

// rpm interrupt service routine
void hall_ISR() {
    pulses++;
}

/**
 *  SETUP
 * */
void setup() {
    // rpm hall interrupt
    pinMode(PIN_HALL_IT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_IT), hall_ISR, RISING);

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

    // Start CAN
    uint8_t error = can.begin(CAN_SPEED, CAN_CONTROLLER_SPEED);
    DEBUG_SERIAL_LN("CAN INIT: " + getCanError(error));
    DEBUG_SERIAL_LN();

    // Set DAC to 0 and save it in memory
    dac.setVoltage(0, true);

}

/**
 *  LOOP
 * */
void loop() {

    if ((millis() - lastUpdate) >= kTelemetryIntervalMs) {
        lastUpdate = millis();

        noInterrupts();
        uint32_t pulses_snapshot = pulses;
        pulses = 0;
        interrupts();

        rpm = (pulses_snapshot / ((kTelemetryIntervalMs / 1000.0f) * 5.0f)) * 60.0f;
        filteredRpm = lowPassFilter(filteredRpm, rpm, kRpmFilterAlpha);
        printTelemetry();
    }

    // Listen for CAN messages
    if (can.checkReceive() == CAN_MSGAVAIL) {
        CanMessage message;
        message.dataLength = 0;
        can.readMsgBuf(&message.dataLength, message.data); 
        message.id = can.getCanId();

        // CURRENT SENSING!!!!!!!
        if (message.id == CAN_ORIONBMS_PACK && message.dataLength >= 4) {
            int16_t rawCurrent = (int16_t)((message.data[2] << 8) | message.data[3]);
            packCurrent = rawCurrent / 10.0f;
            if (hasCurrentReading) {
                filteredCurrent = lowPassFilter(filteredCurrent, packCurrent, kCurrentFilterAlpha);
            } else {
                filteredCurrent = packCurrent;
                hasCurrentReading = true;
            }
        }

        // Debug all received messages to serial monitor
        if(CAN_DEBUG_RECEIVE) {

            Serial.println("-----------------------------");
            Serial.print("CAN MESSAGE RECEIVED - ID: 0x");
            Serial.println(message.id, HEX);

            for (int i = 0; i < message.dataLength; i++) {
                Serial.print("0x");
                Serial.print(message.data[i], HEX);
                Serial.print("\t");
            }
            Serial.println();
        }

    }

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
