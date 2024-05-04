#include "Action.h"

MoveAction::MoveAction(VectorOriented destination, bool isOnlyRotation, bool isBackward):
    destination(destination),
    isOnlyRotation(isOnlyRotation),
    isBackward(isBackward)
{
    id = MOVE;
    hasStarted = false;
    movementDone = false;
}

bool MoveAction::checkClearPath(float distance, float angle) {
    return false;   // TODO
}

void MoveAction::run(float dt, Robot* robot) {
    if (!hasStarted) {
        robot->startMovement(destination, isOnlyRotation, isBackward);
        hasStarted = true;
    }
    robot->updateMovement();
    if (robot->movementDone()) {
        movementDone = true;
    }
}

bool MoveAction::isDone() {
    return movementDone;
}


StaticAction::StaticAction(actionType id) {
    id = id;
    hasStarted = false;
    movementDone = false;
}

bool StaticAction::checkClearPath(float distance, float angle) {
    return true;    // as it is a static action, we do not care of the enemies
}

void StaticAction::run(float dt, Robot* robot) {
    if (!hasStarted) {
        switch (id) {
            case OPEN_CLAWS:
                robot->openClaws();
                break;
            case CLOSE_CLAWS:
                robot->closeClaws();
                break;
            case RAISE_CLAWS:
                robot->raiseClaws();
                break;
            case LOWER_CLAWS:
                robot->lowerClaws();
                break;
            default:
                break;
        }
        hasStarted = true;
    }
    movementDone = true;    // action is considered to do not have any duration
}

bool StaticAction::isDone() {
    return movementDone;
}

DelayAction::DelayAction(unsigned long msduration):
    msduration(msduration)
{
    msstart = 0;
    id = DELAY;
}

bool DelayAction::checkClearPath(float distance, float angle) {
    return true;
}

void DelayAction::run(float dt, Robot* robot) {
    if (!msstart) msstart = millis(); 
}

bool DelayAction::isDone() {
    return ((millis() - msstart) >= msduration);
}
