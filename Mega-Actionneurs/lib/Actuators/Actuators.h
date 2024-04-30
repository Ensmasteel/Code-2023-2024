#ifndef ACTUATORS_H_
#define ACTUATORS_H_

#include "Arduino.h"
#include "Communication.h"
#include "IDS.h"
#include "Servo.h"
#include "pinSetup.h"
//#include "Actuators_Manager.h"



/**
 * Enum class to give the different states of an Actuator.
 */
/**
enum class Actuator_State
{
    Attente, ///< Waiting for an order
    NewMess, ///< getting a new order
    Moving, ///< During the movement
    MoveFinished ///< Just ending the action asked

};*/


class Claw
{
public:

    /**
    * Claw constructor
    */
    Claw();
    
    void closeClaw();
    void openClaw();
    void init();

protected :

    int servoRClosed = 160;
    int servoROpen = 180;
    int servoLClosed = 20;
    int servoLOpen = 0;
    Servo servoR,servoL;

};


class Elevator{
    public:

        /**
         * Public constructor of the Elevator StepperMotor.
         * @param stepPin : pin number of the step pin.
         * @param dirPin : pin number of the direction pin.
         * @param sleepPin : pin number of the sleep pin.
         * @param pinM0 : pin number of the first motor.
         * @param pinM1 : pin number of the second motor.
         */
        Elevator();

        /**
         * Method to make the motor move.
         * @param delay : duration while the stepper should wait for the next action.
         * @param dest_pos : positionof destination in number of steps
         */
        bool move(int dest_pos);
    
    protected :

        int stepsPerRevolution = 200;
        int delay = 175; //microseconds
        int current_step = 0;
        int pos_up = 0;
        int pos_down = 0;
        int stepPin, dirPin, pinM0, pinM1;
};

class Barrel{

    public :
        /**
         * Public constructor of the Barrel StepperMotor.
         */
        Barrel();

        /**
         * Method to make the motor move.
         * @param steps : number of steps that the stepper has to do.
         * @param delay : duration while the stepper should wait for the next action.
         * @param trigo : false to go clockwise, true to go anticlockwise
         */
        void move(int steps, bool trigo);


        bool moveToBrown();
        bool moveToYellow();
        bool moveToPink();
        bool moveToCherries();
    
    protected :

        int stepsPerRevolution = 200;
        int delay = 2500; //microseconds
        int stepPin, dirPin, pinM0, pinM1;
        int stepsPerBarrelRevolution = 1200;
        int currentSteps = 0;
        int posBrown = 0;
        int posYellow = 335;
        int posPink = 670;
        int posCherries = 2;



};

class Bite
{
public:

    /**
    * Claw constructor
    */
    Bite();
    
    void init();
    void deployBrown();
    void retractBrown();
    void deployPink();
    void retractPink();
    void deployYellow();
    void retractYellow();

protected :

    int servoBClosed = 0;
    int servoBOpen = 0;
    int servoPClosed = 0;
    int servoPOpen = 0;
    int servoYClosed = 0;
    int servoYOpen = 0;
    Servo servoB,servoP,servoY;

};

class Cherries
{
public:

    /**
    * Cherries constructor
    */
    Cherries();
    
    void init();
    void downCherries();
    void middleCherries();
    void upCherries();

protected :

    int servoCDown = 0;
    int servoCMiddle = 0;
    int servoCUp = 0;
    Servo servoC;

};
#endif