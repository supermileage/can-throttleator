#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_MCP4725.h"
#include "can_common.h"

#include "settings.h"

mcp2515_can can(PIN_CAN_CS);
Adafruit_MCP4725 dac;

// Current value from 0-4095 being output to DAC
uint16_t currentOutput = 0;

uint32_t lastUpdate = 0;
uint32_t lastHeartbeat = 0;
uint32_t lastLedFlash = 0;

bool ledFlash = false;

uint16_t outputSamples[DATA_POINTS] = {0};

int sampleIndex = 0;
uint32_t rollingSum = 0;

uint32_t periodBuffer[RPM_FILTER_SAMPLES] = {0};
uint8_t periodIndex = 0;
uint32_t periodSum = 0;

uint32_t lastRPM = 0;

volatile uint32_t lastEdge = 0;
float rpm = 0;

float requestedDutyPct = 0.0f;
float appliedDutyPct = 0.0f;

float packCurrentRawA = 0.0f;
float packCurrentFiltA = 0.0f;
bool packCurrentValid = false;
uint32_t lastCurrentRxMs = 0;

static uint32_t lastDebugMs = 0;

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

void hallISR() {
    uint32_t now = micros();
    uint32_t period = now - lastEdge;
    lastEdge = now;

    if (period == 0) return;

    periodSum -= periodBuffer[periodIndex];
    periodSum += period;

    periodBuffer[periodIndex] = period;
    periodIndex = (periodIndex + 1) % RPM_FILTER_SAMPLES;
}

/**
 * @brief Scales raw throttle input by squaring it, and then dividing by scaling factor.
 *          This has the effect of applying a simple curve, such that the throttle is not 
 *          very sensitive on initial press.
 * 
 * @param dutyPct Dutycycle percentage from 0-255
 * @return void
 * */
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
    float driveCurrentRaw  = max(rawCurrent, 0.0f);
    float driveCurrentFilt = max(filtCurrent, 0.0f);

    requestedDuty = constrain(requestedDuty, 0.0f, 100.0f);

    // never received current data, so disable throttle as safety mechanism
    if (!currentValid) return 0.0f;

    // get rid of stale messages
    if ((millis() - lastRxMs) > CURRENT_TIMEOUT_MS) return 0.0f;

    // instantly shut down throttle on hard limit, using raw current
    if (driveCurrentRaw >= HARD_CURRENT_LIMIT_A) return 0.0f;

    // allow full throttle if current under soft limit, using filtered current
    if (driveCurrentFilt <= SOFT_CURRENT_LIMIT_A) return requestedDuty;

    // how to scale the throttle between the hard and soft limits
    float scale = (HARD_CURRENT_LIMIT_A - driveCurrentFilt) / (HARD_CURRENT_LIMIT_A - SOFT_CURRENT_LIMIT_A);

    scale = constrain(scale, 0.0f, 1.0f);

    return requestedDuty * scale;
}

static float filterThrottle(uint8_t rawThrottle) {
    uint16_t scaled = (uint16_t)((rawThrottle / 255.0f) * 100.0f);

    rollingSum -= outputSamples[sampleIndex];
    rollingSum += scaled;

    outputSamples[sampleIndex] = scaled;
    sampleIndex = (sampleIndex + 1) % DATA_POINTS;

    return (float)(rollingSum / DATA_POINTS);
}

/**
 *  SETUP
 * */
void setup() {

    pinMode(PIN_HALL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, RISING);

    // Init and turn on LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Init and turn off Heartbeat pin
    pinMode(PIN_HEARTBEAT, OUTPUT);
    digitalWrite(PIN_HEARTBEAT, LOW);

    if(DEBUG_SERIAL_EN) {
        Serial.begin(SERIAL_MONITOR_SPEED);
    }

    DEBUG_SERIAL_LN();
    DEBUG_SERIAL_LN("*************************");      
	DEBUG_SERIAL_LN("    CAN  THROTTLEATOR    ");                                                    
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

    // Start CAN
    uint8_t error = can.begin(CAN_SPEED, CAN_CONTROLLER_SPEED);
    DEBUG_SERIAL_LN("CAN INIT: " + getCanError(error));
    DEBUG_SERIAL_LN();
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

        // throttle messages
        if(message.id == CAN_STEERING_THROTTLE) {
            requestedDutyPct = SMOOTH_THROTTLE ? filterThrottle(message.data[0]) : (message.data[0] / 255.0f) * 100.0f;
            lastUpdate = millis();
            ledFlash = true;
        }

        // current messages
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

    // If the output is not 0 and it's been more than a certain amount of time since the last
    // throttle CAN message, set the output to 0 as a safety measure
    if(requestedDutyPct && (millis() > lastUpdate + STALE_TIME)) {
        requestedDutyPct = 0;
        ledFlash = false;
        digitalWrite(PIN_LED, HIGH);
    }

    if (periodSum > 0) {
        float avgPeriod = (float)periodSum / RPM_FILTER_SAMPLES;
        float freq = 1e6f / avgPeriod;
        rpm = (freq * 60.0f) / POLE_PAIRS;
    }

    if ((micros() - lastEdge) > 500000UL) {
        rpm = 0;
    }

    if (millis() > lastRPM + RPM_SEND_TIME) {
        lastRPM = millis();

        CanMessage msg;
        msg.id = CAN_ORIONBMS_RPM;
        msg.dataLength = 2;

        uint16_t rpmInt = (uint16_t)rpm;
        msg.data[0] = (rpmInt >> 8) & 0xFF;
        msg.data[1] = rpmInt & 0xFF;
        can.sendMsgBuf(msg.id, CAN_FRAME, msg.dataLength, msg.data);
    }

    // Send a periodic heartbeat CAN message to let other devices on the CAN bus know we are connected
    if(millis() > lastHeartbeat + HEARTBEAT_TIME) {
        lastHeartbeat = millis();

        CanMessage msg;
        msg.id = CAN_STEERING_READY;
        msg.dataLength = 1;
        msg.data[0] = 0x1;
        uint8_t error = can.sendMsgBuf(msg.id, CAN_FRAME, msg.dataLength, msg.data);
        DEBUG_SERIAL_LN("HEARTBEAT SEND: " + getCanError(error));

        // Toggle the heartbeat GPIO
        digitalWrite(PIN_HEARTBEAT, !digitalRead(PIN_HEARTBEAT));
    }

    // If LED should be flashing, flash it
    if((millis() > lastLedFlash + LED_FLASH_INTERVAL) && ledFlash) {
        lastLedFlash = millis();
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }

    // compute safe throttle
    float protectedDutyPct = computeProtectedDuty(
        requestedDutyPct,
        packCurrentRawA,
        packCurrentFiltA,
        packCurrentValid,
        lastCurrentRxMs
    );

    appliedDutyPct = protectedDutyPct;

    // send throttle to DAC
    writeDutyToDac(appliedDutyPct);

    if ((millis() - lastDebugMs) >= 100) {
        lastDebugMs = millis();

        DEBUG_SERIAL("Requested: ");
        DEBUG_SERIAL(requestedDutyPct);
        DEBUG_SERIAL("%, Applied: ");
        DEBUG_SERIAL(appliedDutyPct);
        DEBUG_SERIAL("%, RPM: ");
        DEBUG_SERIAL((int)rpm);
        DEBUG_SERIAL("%, Raw: ");
        DEBUG_SERIAL(packCurrentRawA);
        DEBUG_SERIAL(" A, Filt: ");
        DEBUG_SERIAL(packCurrentFiltA);
        DEBUG_SERIAL_LN(" A");
    }
}
