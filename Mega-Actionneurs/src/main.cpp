#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>
#include "pinSetup.h"
#include "Communication.h"
#include "Logger.h"

Servo rato;
Stepper elevator(200, PIN_ELEVATOR_STEP, PIN_ELEVATOR_DIR);

Communication comTeensy = Communication(&Serial1);
Message msg;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    Logger::setup(&Serial, &Serial, &Serial, false, false, false);

    rato.attach(PIN_CLAWS);
    rato.write(75);

    elevator.setSpeed(4000);
    elevator.step(4000);
    elevator.setSpeed(1000);
}

void loop() {

    comTeensy.update();

    /* Handle the message from the Teensy */
    if (comTeensy.waitingRX()){
        msg = comTeensy.peekOldestMessage();

        switch(msg.aid) {
            case OpenClaws:
                if (msg.did == Todo) rato.write(75);
                break;
            case CloseClaws:
                if (msg.did == Todo) {
                    rato.write(33);
                    delay(300);
                }
                break;
            case RaiseClaws:
                if (msg.did == Todo) elevator.step(2000);
                break;
            case LowerClaws:
                if (msg.did == Todo) elevator.step(-2000);
                break;
            default:
                break;
        }

        comTeensy.popOldestMessage();
    }

    /* Send a message to the Teensy */
    // Message messSend = newMessageEndAction(Arduino,Teensy,PaletJaune);
    // comTeensy.send(messSend);
}
