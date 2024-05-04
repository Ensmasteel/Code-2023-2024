#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>
#include "pinSetup.h"
#include "Communication.h"
#include "Logger.h"

Servo rato;
Stepper elevator(200, PIN_ELEVATOR_STEP, PIN_ELEVATOR_DIR);
bool elevator_raised;

Communication comTeensy = Communication(&Serial1);
Message msg;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    Logger::setup(&Serial, &Serial, &Serial, false, false, false);

    rato.attach(PIN_CLAWS);
    rato.write(75);

    elevator.setSpeed(3000);
    elevator.step(3500);
    elevator_raised = false;
    elevator.setSpeed(800);
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
                if (msg.did == Todo) rato.write(33);
                break;
            case RaiseClaws:
                if (msg.did == Todo && !elevator_raised) {
                    elevator.step(2000);
                    elevator_raised = true;
                }
                break;
            case LowerClaws:
                if (msg.did == Todo && elevator_raised) {
                    elevator.step(-2000);
                    elevator_raised = false;
                }
                break;
            default:
                break;
        }

        comTeensy.popOldestMessage();
    }
}
