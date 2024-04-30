#include "Actuators.h"

Claw::Claw(){

}
void Claw::init(){
    servoL.attach(PIN_LEFT_SERVO_PWM);
    servoR.attach(PIN_RIGHT_SERVO_PWM);
    openClaw();
}


void Claw::closeClaw(){
    servoR.write(servoRClosed);
    servoL.write(servoLClosed);
}

void Claw::openClaw(){
    servoR.write(servoROpen);
    servoL.write(servoLOpen);
}


Elevator::Elevator() {
    this->stepPin = PIN_ELEVATOR_STEPPER_STEP_PIN;
    this->dirPin = PIN_ELEVATOR_STEPPER_DIR_PIN;
    this->pinM0 = PIN_ELEVATOR_STEPPER_M0_PIN;
    this->pinM1 = PIN_ELEVATOR_STEPPER_M1_PIN;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(pinM0, OUTPUT);
    pinMode(pinM1, OUTPUT);

    digitalWrite(pinM0, LOW);
    digitalWrite(pinM1, HIGH);


}

bool Elevator::move(int dest_pos) {
    int steps;
    bool up;
    if (current_step < dest_pos){
        bool up = true;
        digitalWrite(dirPin, HIGH);
        if (dest_pos - current_step > 1000){
            steps = 1000;
        }
        else {
            steps = dest_pos - current_step;
        }}
    else if (current_step > dest_pos) {
        bool up = false;
        digitalWrite(dirPin, LOW);
        if (current_step - dest_pos > 1000){
            steps = 1000;
        }
        else {
            steps = current_step - dest_pos;
        }} 
    else {
        steps = 0;
    }
     
    for(int x = 0; x < steps; x++) {
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(delay);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(delay);
        if (up){current_step++;}
        else{current_step--;}
    
    }
    return (current_step == dest_pos);
    }
    //digitalWrite(sleepPin, holdPosition ? HIGH : LOW);



Barrel::Barrel() {
    this->stepPin = PIN_BARREL_STEPPER_STEP_PIN;
    this->dirPin = PIN_BARREL_STEPPER_DIR_PIN;
    this->pinM0 = PIN_BARREL_STEPPER_M0_PIN;
    this->pinM1 = PIN_BARREL_STEPPER_M1_PIN;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(pinM0, OUTPUT);
    pinMode(pinM1, OUTPUT);

    digitalWrite(pinM0, LOW);
    digitalWrite(pinM1, LOW);
}

void Barrel::move(int steps, bool trigo) {
    
    digitalWrite(dirPin, trigo ? HIGH : LOW);
    if (trigo)
    {
        for(int x = 0; x < steps; x++) {
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(delay);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(delay);
        currentSteps+=1;
    }}
    else
    {
        for(int x = 0; x < steps; x++) {
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(delay);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(delay);
        currentSteps-=1;
    }
    
}}




bool Barrel::moveToBrown(){
    int movement = posBrown - currentSteps;
    if (movement > stepsPerBarrelRevolution / 2)
    {if (stepsPerBarrelRevolution - movement < 1000){
        this->move(stepsPerBarrelRevolution - movement, false);}
    else {this->move(1000, false);}
    }

    else if (movement < - stepsPerBarrelRevolution / 2)
    {if (stepsPerBarrelRevolution + movement < 1000){
        this->move(stepsPerBarrelRevolution + movement, true);}
        else {this->move(1000, true);}
    }

    else if (movement < 0)
    {if (abs(movement) < 1000){
        this->move(abs(movement), false);}
        else{
            this->move(1000, false);
        }
    }

    else if (movement > 0)
    {if (movement < 1000){
        this->move(movement, true);}
        else{
            this->move(1000, true);
        }
    }
    return(posBrown == currentSteps);
}

bool Barrel::moveToPink(){
    int movement = posPink - currentSteps;
    if (movement > stepsPerBarrelRevolution / 2)
    {if (stepsPerBarrelRevolution - movement < 1000){
        this->move(stepsPerBarrelRevolution - movement, false);}
    else {this->move(1000, false);}
    }

    else if (movement < - stepsPerBarrelRevolution / 2)
    {if (stepsPerBarrelRevolution + movement < 1000){
        this->move(stepsPerBarrelRevolution + movement, true);}
        else {this->move(1000, true);}
    }

    else if (movement < 0)
    {if (abs(movement) < 1000){
        this->move(abs(movement), false);}
        else{
            this->move(1000, false);
        }
    }

    else if (movement > 0)
    {if (movement < 1000){
        this->move(movement, true);}
        else{
            this->move(1000, true);
        }
    }
    return(posPink == currentSteps);
}

bool Barrel::moveToYellow(){
    int movement = posYellow - currentSteps;
    if (movement > stepsPerBarrelRevolution / 2)
    {if (stepsPerBarrelRevolution - movement < 1000){
        this->move(stepsPerBarrelRevolution - movement, false);}
    else {this->move(1000, false);}
    }

    else if (movement < - stepsPerBarrelRevolution / 2)
    {if (stepsPerBarrelRevolution + movement < 1000){
        this->move(stepsPerBarrelRevolution + movement, true);}
        else {this->move(1000, true);}
    }

    else if (movement < 0)
    {if (abs(movement) < 1000){
        this->move(abs(movement), false);}
        else{
            this->move(1000, false);
        }
    }

    else if (movement > 0)
    {if (movement < 1000){
        this->move(movement, true);}
        else{
            this->move(1000, true);
        }
    }
    return(posYellow == currentSteps);
}

bool Barrel::moveToCherries(){
    int movement = posCherries - currentSteps;
    if (movement > stepsPerBarrelRevolution / 2)
    {if (stepsPerBarrelRevolution - movement < 1000){
        this->move(stepsPerBarrelRevolution - movement, false);}
    else {this->move(1000, false);}
    }

    else if (movement < - stepsPerBarrelRevolution / 2)
    {if (stepsPerBarrelRevolution + movement < 1000){
        this->move(stepsPerBarrelRevolution + movement, true);}
        else {this->move(1000, true);}
    }

    else if (movement < 0)
    {if (abs(movement) < 1000){
        this->move(abs(movement), false);}
        else{
            this->move(1000, false);
        }
    }

    else if (movement > 0)
    {if (movement < 1000){
        this->move(movement, true);}
        else{
            this->move(1000, true);
        }
    }
    return(posCherries == currentSteps);
}

Bite::Bite(){



}

void Bite::init(){
    servoB.attach(PIN_BROWN_SERVO_PWM);
    servoP.attach(PIN_PINK_SERVO_PWM);
    servoY.attach(PIN_YELLOW_SERVO_PWM);
    deployBrown();
    deployPink();
    deployYellow();
}

void Bite::deployBrown(){
    servoB.write(servoBOpen);
}

void Bite::deployPink(){
    servoP.write(servoPOpen);
}

void Bite::deployYellow(){
    servoY.write(servoYOpen);
}

void Bite::retractBrown(){
    servoB.write(servoBClosed);
}

void Bite::retractPink(){
    servoP.write(servoPClosed);
}

void Bite::retractYellow(){
    servoY.write(servoYClosed);
}

Cherries::Cherries(){

    

}

void Cherries::init(){
    servoC.attach(PIN_CHERRIES_SERVO_PWM);
    middleCherries();
}

void Cherries::downCherries(){
    servoC.write(servoCDown);
}

void Cherries::middleCherries(){
    servoC.write(servoCMiddle);
}

void Cherries::upCherries(){
    servoC.write(servoCUp);
}