#ifndef ACTION_H
#define ACTION_H

#include "Robot.h"
#include "Vector.h"

enum actionType {
    MOVE,
    DELAY,
    OPEN_CLAWS,
    CLOSE_CLAWS,
    RAISE_CLAWS,
    LOWER_CLAWS
};

class Action {
    public:
        virtual bool checkClearPath(float distance, float angle) = 0;
        virtual void run(float dt, Robot* robot) = 0;
        virtual bool isDone() = 0;

    protected:
        actionType id;
        bool hasStarted;       
        bool movementDone;
};

class MoveAction : public Action {
    public:
        MoveAction(VectorOriented destination, bool isOnlyRotation, bool isBackward);
        bool checkClearPath(float distance, float angle) override;
        void run(float dt, Robot* robot) override;
        bool isDone() override;

    private:
        VectorOriented destination;
        bool isOnlyRotation;
        bool isBackward;
};

class StaticAction : public Action {
    public:
        StaticAction(actionType id);
        bool checkClearPath(float distance, float angle) override;
        void run(float dt, Robot* robot) override;
        bool isDone() override;
};

class DelayAction : public Action {
    public:
        DelayAction(unsigned long msduration);
        bool checkClearPath(float distance, float angle) override;
        void run(float dt, Robot* robot) override;
        bool isDone() override;

    private:
        unsigned long msstart;
        unsigned long msduration;
};


#endif
