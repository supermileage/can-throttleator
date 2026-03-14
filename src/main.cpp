#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_MCP4725.h"
#include "can_common.h"

#include "settings.h"

mcp2515_can can(PIN_CAN_CS);
Adafruit_MCP4725 dac;

uint16_t currentOutput = 0;
static const int kConfirmThreshold = 101;

// This struct contains all the components of a CAN message. dataLength must be <= 8, 
// and the first [dataLength] positions of data[] must contain valid data
typedef uint8_t CanBuffer[8];
struct CanMessage {
        uint32_t id;
        uint8_t dataLength;
        CanBuffer data;
};

/**
 * @brief Converts CAN status message to readable output
 * 
 * @param errorCode CAN status message
 * @return Readable output
 * */
String getCanError(uint8_t errorCode) {
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


float requestedDutyPct = 0.0f;
float appliedDutyPct = 0.0f;

float packCurrentRawA = 0.0f;
float packCurrentFiltA = 0.0f;
bool packCurrentValid = false;
uint32_t lastCurrentRxMs = 0;

const float CURRENT_FILTER_ALPHA = 0.15f;
const float SOFT_CURRENT_LIMIT_A = 40.0f;   // start reducing throttle above this
const float HARD_CURRENT_LIMIT_A = 60.0f;   // force throttle to 0 above this
const uint32_t CURRENT_TIMEOUT_MS = 1000;    // if CAN current is stale, shut down

const float OUTPUT_RAMP_UP_PCT_PER_LOOP   = 0.01f;

static uint32_t lastDebugMs = 0;

static void writeDutyToDac(float dutyPct) {
    static uint16_t lastOutput = 0xFFFF;

    dutyPct = constrain(dutyPct, 0.0f, 100.0f);

    uint16_t newOutput = (uint16_t)((dutyPct / 100.0f) * DAC_MAX_CMD);

    if (newOutput == lastOutput) {
        return;
    }

    currentOutput = newOutput;
    dac.setVoltage(currentOutput, false);
    lastOutput = newOutput;
}

static float computeProtectedDuty(float requestedDuty, float rawCurrent, float filtCurrent, bool currentValid, uint32_t lastRxMs) {
    requestedDuty = constrain(requestedDuty, 0.0f, 100.0f);

    // never received current data, so disable throttle as safety mechanism
    if (!currentValid) {
        return 0.0f;
    }

    // get rid of stale messages
    if ((millis() - lastRxMs) > CURRENT_TIMEOUT_MS) {
        return 0.0f;
    }

    // instantly shut down throttle on hard limit, using raw current
    if (rawCurrent >= HARD_CURRENT_LIMIT_A) {
        return 0.0f;
    }

    // allow full throttle if current under soft limit, using filtered current
    if (filtCurrent <= SOFT_CURRENT_LIMIT_A) {
        return requestedDuty;
    }

    // how to scale the throttle between the hard and soft limits
    float scale = (HARD_CURRENT_LIMIT_A - filtCurrent) / (HARD_CURRENT_LIMIT_A - SOFT_CURRENT_LIMIT_A);

    scale = constrain(scale, 0.0f, 1.0f);

    return requestedDuty * scale;
}

static float slewLimitDuty(float targetDuty, float currentDuty) {
    targetDuty = constrain(targetDuty, 0.0f, 100.0f);
    currentDuty = constrain(currentDuty, 0.0f, 100.0f);

    // if torque needs to decrease, do it immediately
    if (targetDuty < currentDuty) {
        return targetDuty;
    }

    // otherwise ramp up slowly
    currentDuty += OUTPUT_RAMP_UP_PCT_PER_LOOP;

    if (currentDuty > targetDuty) {
        currentDuty = targetDuty;
    }

    return currentDuty;
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

    // Listen for CAN messages
    if (can.checkReceive() == CAN_MSGAVAIL) {
        CanMessage message;
        message.dataLength = 0;
        can.readMsgBuf(&message.dataLength, message.data); 
        message.id = can.getCanId();

        // CURRENT SENSING!!!!!!!
        if (message.id == CAN_ORIONBMS_PACK && message.dataLength >= 4) {
            int16_t rawCurrent = (int16_t)((message.data[2] << 8) | message.data[3]);
            packCurrentRawA = rawCurrent / 10.0f;

            // bootstrap filter with valid value
            if (!packCurrentValid) {
                packCurrentFiltA = packCurrentRawA;
            } else {
                // EMA filtering
                packCurrentFiltA += CURRENT_FILTER_ALPHA * (packCurrentRawA - packCurrentFiltA);
            }
            packCurrentValid = true;
            lastCurrentRxMs = millis();
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
                requestedDutyPct = duty;
                DEBUG_SERIAL_LN("Requested dutycycle set to: " + String(requestedDutyPct) + "%");
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

    // compute safe throttle
    float protectedDutyPct = computeProtectedDuty(
        requestedDutyPct,
        packCurrentRawA,
        packCurrentFiltA,
        packCurrentValid,
        lastCurrentRxMs
    );

    // smooth throttle changes
    appliedDutyPct = slewLimitDuty(protectedDutyPct, appliedDutyPct);

    // send throttle to DAC
    writeDutyToDac(appliedDutyPct);

    if ((millis() - lastDebugMs) >= 100) {
        lastDebugMs = millis();

        DEBUG_SERIAL("Requested: ");
        DEBUG_SERIAL(requestedDutyPct);
        DEBUG_SERIAL("%, Applied: ");
        DEBUG_SERIAL(appliedDutyPct);
        DEBUG_SERIAL("%, Raw: ");
        DEBUG_SERIAL(packCurrentRawA);
        DEBUG_SERIAL(" A, Filt: ");
        DEBUG_SERIAL(packCurrentFiltA);
        DEBUG_SERIAL_LN(" A");
    }
}
