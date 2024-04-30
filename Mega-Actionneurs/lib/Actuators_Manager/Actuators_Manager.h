#ifndef ACTUATORS_MANAGER_H_
#define ACTUATORS_MANAGER_H_

#include "Actuators.h"
#include "Communication.h"
#include "IDS.h"





class Actuator_Manager {
    public :
        Actuator_Manager();
        void init();

        Actuator_Manager(Communication * comTeensy);
        uint8_t getIdAction();
        void setIdAction(uint8_t id);
        void setBoolAction(bool act);
        void setWaitMessage(bool wm);
        void updateCommunication();
        bool updateStop();

    private :
        int dest_up = 12000;
        int dest_down = 0;
        Communication * comTeensy;
        uint8_t idActionInProgress = 0;
        bool actionInProgress = false;
        bool waitMessage = false;
        Elevator elevator = Elevator();
        Claw claw = Claw();
        Barrel barrel = Barrel();
        Bite bite = Bite();
        Cherries cherries = Cherries();
        bool stop = false;
        bool finished= true;
        ActionID aid;
};

#endif