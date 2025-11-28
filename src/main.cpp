#include <Arduino.h>
#include <SPI.h>

#include "mcp2515_can.h"
#include "can_common.h"
#include "settings.h"

// same PIN_CAN_CS, CAN_SPEED, CAN_CONTROLLER_SPEED, CAN_FRAME as your other code

const int PIN_POT = A0;
const unsigned long TX_PERIOD_MS = 1;  // slower for readability

mcp2515_can can(PIN_CAN_CS);
unsigned long lastTx = 0;

String getCanError(uint8_t errorCode){
    switch(errorCode){
        case CAN_OK:             return "CAN OK";
        case CAN_FAILINIT:       return "CAN FAIL INIT";
        case CAN_FAILTX:         return "CAN FAIL TX";
        case CAN_MSGAVAIL:       return "CAN MSG AVAIL";
        case CAN_NOMSG:          return "CAN NO MSG";
        case CAN_CTRLERROR:      return "CAN CTRL ERROR";
        case CAN_GETTXBFTIMEOUT: return "CAN TX BF TIMEOUT";
        case CAN_SENDMSGTIMEOUT: return "CAN SEND MSG TIMEOUT";
        default:                 return "CAN FAIL";
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_POT, INPUT);

    Serial.println("Nano CAN sender starting...");

    uint8_t initErr = can.begin(CAN_SPEED, CAN_CONTROLLER_SPEED);
    Serial.print("can.begin() -> ");
    Serial.print(initErr);
    Serial.print(" (");
    Serial.print(getCanError(initErr));
    Serial.println(")");

    if (initErr != CAN_OK) {
        Serial.println("CAN init FAILED, stopping here.");
        while (1) { delay(1000); }
    }
}

void loop()
{
    if (millis() - lastTx >= TX_PERIOD_MS) {
        lastTx = millis();

        int raw = analogRead(PIN_POT);
        uint8_t throttle = map(raw, 0, 1023, 0, 255);

        byte data[8] = {0};
        data[0] = throttle;

        uint8_t err = can.sendMsgBuf(CAN_STEERING_THROTTLE, CAN_FRAME, 1, data);

        Serial.print("TX throttle=");
        Serial.print(throttle);
        Serial.print(" -> err=");
        Serial.print(err);
        Serial.print(" (");
        Serial.print(getCanError(err));
        Serial.println(")");
    }
}
