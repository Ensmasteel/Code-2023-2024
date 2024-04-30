#include "Robot.h"

#define SYM 0.001

Robot::Robot(float x_ini, float y_ini, float theta_ini){

    this->vectIni=VectorOriented(x_ini,y_ini,theta_ini);
    //vectIni=VectorOriented(0.725,1.125,PI/4);
    this->kineticCurrent=Kinetic(x_ini,y_ini,theta_ini,0,0);
    this->kineticNext=Kinetic(x_ini,y_ini,theta_ini,0,0);

    this->ghost = Ghost(vectIni);
    this->motorL =  Motor(PIN_LEFT_MOTOR_PWM, PIN_LEFT_MOTOR_IN1, 13, true); 
    this->motorR = Motor(PIN_RIGHT_MOTOR_PWM, PIN_RIGHT_MOTOR_IN1, 13, false);

    this->codeuseR = Codeuse(PIN_CODEUSE_DROITE_A, PIN_CODEUSE_DROITE_B, 8192*(1-SYM), 0.0566 , true);
    this->codeuseL = Codeuse(PIN_CODEUSE_GAUCHE_A, PIN_CODEUSE_GAUCHE_B, 16384*(1+SYM), 0.0566 , true);
  
    this->switchL = Switch(18);
    this->switchR = Switch(19);

    this->odometry = Odometry(&codeuseL,&codeuseR,0.2555,&switchL,&switchR,&kineticCurrent);

    this->controller = Asservissement(&translationOrder, &rotationOrder, &kineticCurrent, &kineticNext, 100.0);


}

void Robot::updateMovement(){

    //odometry.updateOdometry(1.0 / 83.33);  // This call updates cinetiqueCurrent
    ghost.actuatePosition(1.0 / 100.0);
    kineticNext = ghost.getControllerKinetic();
    //kineticNext.printDebug("Ghost");
    controller.compute(1.0 / 100.0);      // This call updates translationOrder and rotationOrder
    /*Serial.println("consigne");
    Serial.println(">TOrder :" + String (translationOrder));
    
    Serial.println(">ROrder :" + String (rotationOrder)); */
    //kineticCurrent.printDebug("Robot");
    motorL.setPWMValue(-(translationOrder - rotationOrder));
    motorL.actuate();
    motorR.setPWMValue((translationOrder + rotationOrder));
    motorR.actuate();
}

void Robot::startMovement(VectorOriented nextDest,bool isOnlyRotation,bool isBackward){
    Serial.println("Compute");
    
    ghost.computeTrajectory(nextDest,0.3, MoveProfilesSetup::get(standard, !isOnlyRotation)->speedRamps, MoveProfilesSetup::get(standard, !isOnlyRotation)->cruisingSpeed, isOnlyRotation, isBackward);
    ghost.setLock(false);
    inMove=true;
    startActionMillis=millis();
}

/*void Robot::startMovementBackwardDepot(VectorOriented nextDest){
    Serial.println("compute");
    ghost.computeTrajectory(nextDest,0.3, MoveProfilesSetup::get(standard, true)->speedRamps, MoveProfilesSetup::get(standard, true)->cruisingSpeed, false, true);
    ghost.setLock(false);
    startActionMillis=millis();
}

void Robot::startMovementRecallageRotation(VectorOriented nextDest){
    ghost.computeTrajectory(nextDest,0.3, MoveProfilesSetup::get(standard, false)->speedRamps, MoveProfilesSetup::get(standard, false)->cruisingSpeed, true, false);
    ghost.setLock(false);
    startActionMillis=millis(); 
}*/

bool Robot::endMovement(){
    bool out = ghost.getTrajectoryIsFinished() && controller.close;
    if (out){
        ghost.goToRobot(kineticCurrent);
        inMove=false;
        //Serial.println("Mouvement termine");
    }
    else if (!out && (millis()-startActionMillis)>10000){
        Serial.println("Mouvement failed et arrete");
        out=true;
        inMove=false;
        ghost.goToRobot(kineticCurrent);
    }
    else{}
    return out;
}

void Robot::resumeMotor(){
    motorL.resume();
    motorR.resume();

}

void Robot::stopMovement(){
    motorL.stop();
    motorR.stop();


    motorL.actuate();
    motorR.actuate();
}
