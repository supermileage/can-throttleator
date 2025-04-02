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

// RPM Measurements
volatile unsigned int count = 0; // Counter for hall sensor triggers
unsigned long previousMillis = 0; // Time variable for interval calculation
int rpm; // 0-9000, Rotations Per Minute of the motor

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

/**
 * @brief Scales raw throttle input by squaring it, and then dividing by scaling factor.
 *          This has the effect of applying a simple curve, such that the throttle is not 
 *          very sensitive on initial press.
 * 
 * @param input Throttle value from 0-255
 * @return Voltage value from 0-4095
 * */
uint16_t scaleThrottle(uint8_t input) {
    uint32_t output = input + 1;

    if(SCALE_EXP) {
        output = (output * output) / DAC_VALUE_TO_INPUT;
    } else {
        output = output * 16;
    }

    if(output) {
        output--;
    }

    return output;
}

void countPulse() {
  // Increment count when hall sensor triggers
  count++;
  }

/**
 *  SETUP
 * */
void setup() {

    // Init and turn on LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Start Serial Monitor
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

    // Initialize interrupt for RPM Measurement
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_A), countPulse, RISING); // Attach interrupt to hall sensor pin
}

/**
 *  LOOP
 * */
void loop() {

    uint16_t newOutput = currentOutput;

    // Listen for CAN messages
    if (can.checkReceive() == CAN_MSGAVAIL) {
        CanMessage message;
        message.dataLength = 0;
        can.readMsgBuf(&message.dataLength, message.data); 
        message.id = can.getCanId();

        // If a new throttle message is received, scale it
        if(message.id == CAN_STEERING_THROTTLE) {
            newOutput = scaleThrottle(message.data[0]);
            lastUpdate = millis();
            ledFlash = true;
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

    // RPM Measurement
    unsigned long currentMillis = millis();

    // Calculate RPM every interval
    if (currentMillis - previousMillis >= TIME_INTERVAL) {
        // Calculate RPM (revolutions per minute)
        rpm = (count*60000 / (float(TIME_INTERVAL))); // 60000 milliseconds in a minute
        rpm = rpm/3.5;
        // Print RPM
        DEBUG_SERIAL("RPM: ");
        DEBUG_SERIAL_LN(rpm);

        // Send RPM to CAN
        byte dataBytes[2];
        dataBytes[0] = (rpm >> 8) & 0xFF; // Most significant byte
        dataBytes[1] = rpm & 0xFF;        // Least significant byte
        can.sendMsgBuf(CAN_MOTOR_RPM,0,2,dataBytes);

        // Reset counter and timer
        count = 0;
        previousMillis = currentMillis;
    }

    // If the new output is different than the old output, update the voltage on the DAC
    if(newOutput != currentOutput) {

        // Limits throttle until RPM reaches Minimum RPM
        if(RPM_LIMITER){
            if((rpm < MIN_RPM) && (newOutput > MIN_THROTTLE)){
                    newOutput = MIN_THROTTLE;
            }
        }

        rollingSum -= outputSamples[sampleIndex];
        rollingSum += newOutput;
        outputSamples[sampleIndex] = newOutput;
        sampleIndex = (sampleIndex + 1) % DATA_POINTS;

        // Calculate the rolling average
        uint16_t rollingAverage = rollingSum / DATA_POINTS;
        
        currentOutput = rollingAverage;
    }

    dac.setVoltage(currentOutput, false); 
    //DEBUG_SERIAL_LN("Voltage set to: " + String(currentOutput / DAC_VALUE_TO_V) + "v");

    // If the output is not 0 and it's been more than a certain amount of time since the last
    // throttle CAN message, set the output to 0 as a safety measure
    if(currentOutput && (millis() > lastUpdate + STALE_TIME)) {
        currentOutput = 0;
        dac.setVoltage(currentOutput, false);
        DEBUG_SERIAL_LN("ERROR: NO DATA - Output set to 0v");
        ledFlash = false;
        
        digitalWrite(PIN_LED, HIGH);
    }

    // Send a periodic heartbeat CAN message to let other devices on the CAN bus know we are connected
    if(millis() > lastHeartbeat + HEARTBEAT_TIME) {
        lastHeartbeat = millis();

        CanMessage msg;
        msg.id = THROTTLE_HEARTBEAT;
        msg.dataLength = 1;
        msg.data[0] = 0x1;
        uint8_t error = can.sendMsgBuf(msg.id, CAN_FRAME, msg.dataLength, msg.data);
        DEBUG_SERIAL_LN("HEARTBEAT SEND: " + getCanError(error));
    }

    // If LED should be flashing, flash it
    if((millis() > lastLedFlash + LED_FLASH_INTERVAL) && ledFlash) {
        lastLedFlash = millis();
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }

}
