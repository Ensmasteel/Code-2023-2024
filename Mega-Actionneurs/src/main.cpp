#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>
#include "pinSetup.h"
#include "Communication.h"
#include "Logger.h"

Servo rato;
Stepper elevator(200, PIN_ELEVATOR_STEP, PIN_ELEVATOR_DIR);
Servo SolarLeft;
Servo SolarRight;
bool elevator_raised;
bool IsSolarLeft;
bool IsSolarRight;

Communication comTeensy = Communication(&Serial1);
Message msg;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    Logger::setup(&Serial, &Serial, &Serial, false, false, false);

    rato.attach(PIN_CLAWS);
    rato.write(75);

    SolarLeft.attach(PIN_SOLARLEFT);
    SolarRight.attach(PIN_SOLARRIGHT);

    elevator.setSpeed(3000);
    elevator.step(3500);
    elevator_raised = false;
    IsSolarLeft = false;
    IsSolarRight = false;
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
            case StartMagnet:
                if (msg.did == Todo) digitalWrite(PIN_MAGNET,HIGH);
                break;
            case ShutdownMagnet:
                if (msg.did == Todo) digitalWrite(PIN_MAGNET,LOW);
                break;
            case SolarLeftOn:
                if (msg.did == Todo && !IsSolarRight) {
                    //SolarLeft.write(90);
                    IsSolarLeft = true;
                }
                break;
            case SolarLeftOff:
                if (msg.did == Todo) {
                    //SolarLeft.write(0);
                    IsSolarLeft = false;
                }  
                break;
            case SolarRightOn:
                if (msg.did == Todo && !IsSolarRight) {
                    //SolarRight.write(90);
                    IsSolarRight = true;
                }
                break;
            case SolarRightOff:
                if (msg.did == Todo) {
                    //SolarRight.write(0);
                    IsSolarRight = false;
                }
                break;
            default:
                break;
        }
        comTeensy.popOldestMessage();
    }
}
