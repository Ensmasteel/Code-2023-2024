#include "Actuators_Manager.h"

Actuator_Manager::Actuator_Manager(){}




void Actuator_Manager::init(){
    claw.init();
    bite.init();
    cherries.init();
}



Actuator_Manager::Actuator_Manager(Communication * comTeensy){
    this->comTeensy=comTeensy;
}

uint8_t Actuator_Manager::getIdAction(){
    return idActionInProgress;
}

void Actuator_Manager::setIdAction(uint8_t id){
    this->idActionInProgress=id;
}

void Actuator_Manager::setBoolAction(bool act){
    this->actionInProgress=act;
}

void Actuator_Manager::setWaitMessage(bool wm){
    this->waitMessage=wm;
}

void Actuator_Manager::updateCommunication(){
    /*if(comTeensy->waitingRX()){
        Message currentMessage = comTeensy->peekOldestMessage();
        DataID did = currentMessage.did;
        ActionID aid = currentMessage.aid;

        switch(did){

            case Todo : {*/
                stop = false;
                switch(aid){

                    case PaletMarron : { //Recupere un palet marron et stocke dans la position marron
                        if (!stop){
                        claw.closeClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = barrel.moveToBrown();
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_up);
                        stop = !updateStop();
                        }

                        if (!stop){
                        bite.deployBrown();
                        claw.openClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_down);
                        stop = !updateStop();
                        }

                        Message mess;
                        mess = newMessageEndAction(Arduino,Teensy,aid);
                        comTeensy->send(mess);
                        waitMessage = true;
                        actionInProgress=false;
                    }

                    case PaletJaune : { //Recupere un palet jaune et stocke dans la position jaune
                        if (!stop){
                        claw.closeClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = barrel.moveToYellow();
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_up);
                        stop = !updateStop();
                        }

                        if (!stop){
                        bite.deployYellow();
                        claw.openClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_down);
                        stop = !updateStop();
                        }

                        Message mess;
                        mess = newMessageEndAction(Arduino,Teensy,aid);
                        comTeensy->send(mess);
                        waitMessage = true;
                        actionInProgress=false;
                    }

                    case PaletRose : { //Recupere un palet rose et stocke dans la position rose
                        if (!stop){
                        claw.closeClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = barrel.moveToPink();
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_up);
                        stop = !updateStop();
                        }

                        if (!stop){
                        bite.deployPink();
                        claw.openClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_down);
                        stop = !updateStop();
                        }

                        Message mess;
                        mess = newMessageEndAction(Arduino,Teensy,aid);
                        comTeensy->send(mess);
                        waitMessage = true;
                        actionInProgress=false;
                    }

                    case Depot : { //Construit un (et un seul) gateau
                        bool finished = false;
                        while (!stop && !finished){
                        finished = barrel.moveToBrown();
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_up);
                        stop = !updateStop();
                        }

                        if (!stop){
                        claw.closeClaw();
                        bite.retractBrown();
                        bite.deployBrown();
                        claw.openClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_down);
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = barrel.moveToYellow();
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_up);
                        stop = !updateStop();
                        }

                        if (!stop){
                        claw.closeClaw();
                        bite.retractYellow();
                        bite.deployYellow();
                        claw.openClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_down);
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = barrel.moveToPink();
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_up);
                        stop = !updateStop();
                        }

                        if (!stop){
                        claw.closeClaw();
                        bite.retractPink();
                        bite.deployPink();
                        claw.openClaw();
                        stop = !updateStop();}

                        finished = false;
                        while (!stop && !finished){
                        finished = elevator.move(dest_down);
                        stop = !updateStop();
                        }

                        finished = false;
                        while (!stop && !finished){
                        finished = barrel.moveToCherries();
                        stop = !updateStop();
                        }

                        if (!stop){
                        cherries.upCherries();
                        cherries.downCherries();
                        cherries.middleCherries();
                        stop = !updateStop();}

                        Message mess;
                        mess = newMessageEndAction(Arduino,Teensy,aid);
                        comTeensy->send(mess);
                        waitMessage = true;
                        actionInProgress=false;
                    }
                default:
                    break;
    
                }
                comTeensy->popOldestMessage();
            }
        //default :
        //    break;
        
//}
//}
//}

bool Actuator_Manager::updateStop(){
    return(true);
    /*
    if(comTeensy->waitingRX()){
        Message currentMessage = comTeensy->peekOldestMessage();
        DataID did = currentMessage.did;
        comTeensy->popOldestMessage();
        return (did == EndAction);
}*/}
