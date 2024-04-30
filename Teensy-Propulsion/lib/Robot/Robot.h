#ifndef ROBOT_H_
#define ROBOT_H_

#include "Vector.h"
#include "Ghost.h"
#include "Asservissement.h"
#include "Motor.h"
#include "pinSetup.h"
#include "Odometry.h"

class Robot{

    public :

        Robot(float x_ini, float y_ini, float theta_ini);
        Robot(){};

        bool getInMove(){return inMove;};
        void updateMovement();
        void startMovement(VectorOriented nextDest,bool isOnlyRotation,bool isBackward);
        /*void startMovementBackwardDepot(VectorOriented nextDest);
        void startMovementRecallageRotation(VectorOriented nextDest);*/
        bool endMovement();
        void stopMovement();
        void resumeMotor();

        Kinetic kineticCurrent, kineticNext;
        Ghost ghost;

        Odometry odometry;
    private :

        //Kinetic kineticCurrent, kineticNext;
        
        Asservissement controller;
        Codeuse codeuseL, codeuseR;
        Switch switchL,switchR;

        bool inMove = false;


        Motor motorL,motorR;
        VectorOriented vectIni;

        float translationOrder, rotationOrder;
        

        float startActionMillis;


};

#endif