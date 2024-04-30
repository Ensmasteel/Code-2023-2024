#ifndef ROBOT_H_
#define ROBOT_H_

#include "Asservissement.h"
#include "Ghost.h"
#include "Motor.h"
#include "Odometry.h"
#include "Vector.h"
#include "pinSetup.h"

class Robot {
   public:
    Robot(float x_ini, float y_ini, float theta_ini);
    Robot(){};

    bool getInMove() { return inMove; };
    void updateMovement();
    void startMovement(VectorOriented nextDest, bool isOnlyRotation, bool isBackward);
    /*void Robot::startMovementBackwardDepot(VectorOriented nextDest);
    void Robot::startMovementRecallageRotation(VectorOriented nextDest);*/
    bool endMovement();
    void stopMovement();
    void resumeMotor();

    Kinetic kineticCurrent, kineticNext;
    Ghost ghost;

    Odometry odometry;

   private:
    Asservissement controller;
    Codeuse codeuseL, codeuseR;
    Switch switchL, switchR;

    bool inMove = false;

    Motor motorL, motorR;
    VectorOriented vectIni;

    float translationOrder, rotationOrder;

    float startActionMillis;
};

#endif
