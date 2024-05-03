#include <Arduino.h>
#include "Servo.h"
#include "pinSetup.h"
#include "Communication.h"
#include "Logger.h"

Servo rato, panneaux;

Communication comTeensy = Communication(&Serial1);
Message msg;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    Logger::setup(&Serial, &Serial, &Serial, false, false, false);

    pinMode(PIN_ELEVATOR_STEP, OUTPUT);
    pinMode(PIN_ELEVATOR_DIR, OUTPUT);

    digitalWrite(PIN_ELEVATOR_DIR, HIGH);
    for(int x = 0; x < 50; x++) {
        digitalWrite(PIN_ELEVATOR_STEP,HIGH);
        delayMicroseconds(500);
        digitalWrite(PIN_ELEVATOR_STEP,LOW);
        delayMicroseconds(500);
    }

}

void loop() {

    comTeensy.update();

    /* Handle the message from the Teensy */
    // if (comTeensy.waitingRX()){
    //     msg = comTeensy.peekOldestMessage();

    //     switch(msg.aid) {
    //         case OpenClaws:
    //             break;
    //         case CloseClaws:
    //             break;
    //         case RaiseClaws:
    //             if (msg.did == Todo) {
    //                 digitalWrite(PIN_ELEVATOR_DIR, HIGH);
    //                 for(int x = 0; x < 50; x++) {
    //                     digitalWrite(PIN_ELEVATOR_STEP,HIGH);
    //                     delayMicroseconds(500);
    //                     digitalWrite(PIN_ELEVATOR_STEP,LOW);
    //                     delayMicroseconds(500);
    //                 }
    //             }
    //             break;
    //         case LowerClaws:
    //             break;
    //         default:
    //             break;
    //     }

    //     comTeensy.popOldestMessage();
    // }

    /* Send a message to the Teensy */
    // Message messSend = newMessageEndAction(Arduino,Teensy,PaletJaune);
    // comTeensy.send(messSend);
}
