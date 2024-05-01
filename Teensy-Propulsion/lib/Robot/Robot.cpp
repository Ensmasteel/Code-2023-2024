#include "Robot.h"

#define SYM 0.001

Robot::Robot(float x_ini, float y_ini, float theta_ini) {
    this->vectIni = VectorOriented(x_ini, y_ini, theta_ini);
    this->kineticCurrent = Kinetic(x_ini, y_ini, theta_ini, 0, 0);
    this->kineticNext = Kinetic(x_ini, y_ini, theta_ini, 0, 0);

    this->ghost = Ghost(vectIni);
    this->motorL = Motor(PIN_LEFT_MOTOR_PWM, PIN_LEFT_MOTOR_IN1, 13, true);
    this->motorR = Motor(PIN_RIGHT_MOTOR_PWM, PIN_RIGHT_MOTOR_IN1, 13, false);

    this->codeuseR = Codeuse(PIN_CODEUSE_DROITE_A, PIN_CODEUSE_DROITE_B, 8192 * (1 - SYM), 0.0566, true);
    this->codeuseL = Codeuse(PIN_CODEUSE_GAUCHE_A, PIN_CODEUSE_GAUCHE_B, 16384 * (1 + SYM), 0.0566, true);

    this->switchL = Switch(PIN_SWITCH_L);
    this->switchR = Switch(PIN_SWITCH_R);

    this->odometry = Odometry(&codeuseL, &codeuseR, 0.2555, &switchL, &switchR, &kineticCurrent);

    this->controller = Asservissement(&translationOrder, &rotationOrder, &kineticCurrent, &kineticNext, 100.0);

    this->comMega = Communication(&Serial1);
    this->comESP = Communication(&Serial2);

    this->tirette = Tirette(PIN_TIRETTE);

}

void Robot::updateMovement() {
    ghost.actuatePosition(1.0 / 100.0);
    kineticNext = ghost.getControllerKinetic();
    controller.compute(1.0 / 100.0);  // This call updates translationOrder and rotationOrder
    motorL.setPWMValue(-(translationOrder - rotationOrder));
    motorL.actuate();
    motorR.setPWMValue((translationOrder + rotationOrder));
    motorR.actuate();
}

void Robot::startMovement(VectorOriented nextDest, bool isOnlyRotation, bool isBackward) {
    ghost.computeTrajectory(nextDest, 0.3, MoveProfilesSetup::get(standard, !isOnlyRotation)->speedRamps, MoveProfilesSetup::get(standard, !isOnlyRotation)->cruisingSpeed, isOnlyRotation, isBackward);
    ghost.setLock(false);
    startActionMillis = millis();
}

bool Robot::movementDone() {
    bool out = ghost.getTrajectoryIsFinished() && controller.close;
    if (out) {
        ghost.goToRobot(kineticCurrent);
    } else if ((millis() - startActionMillis) > 10000) {
        Serial.println("Mouvement failed et arrete");
        out = true;
        ghost.goToRobot(kineticCurrent);
    } else {
    }
    return out;
}

void Robot::resumeMotor() {
    motorL.resume();
    motorR.resume();
}

void Robot::stopMovement() {
    motorL.stop();
    motorR.stop();

    motorL.actuate();
    motorR.actuate();
}

void Robot::updateOdometry(float dt) {
    odometry.updateOdometry(dt);
}

bool Robot::testTirette() {
    return tirette.testTirette();
}
