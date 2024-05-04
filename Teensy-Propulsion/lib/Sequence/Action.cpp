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


StaticAction::StaticAction(actionType aid, bool noDuration):
    noDuration(noDuration)
{
    id = aid;
    msstart = 0;
    if (!noDuration) {
        switch (id) {
            case OPEN_CLAWS:
                msduration = 300;
                break;
            case CLOSE_CLAWS:
                msduration = 300;
                break;
            case RAISE_CLAWS:
                msduration = 800;
                break;
            case LOWER_CLAWS:
                msduration = 800;
                break;
            default:
                msduration = 0;
                break;
        }
    } else {
        msduration = 0;
    }
}

bool StaticAction::checkClearPath(float distance, float angle) {
    return true;    // as it is a static action, we do not care of the enemies
}

void StaticAction::run(float dt, Robot* robot) {
    if (!msstart) {
        msstart = millis();
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
    }
}

bool StaticAction::isDone() {
    return ((millis() - msstart) >= msduration);
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
