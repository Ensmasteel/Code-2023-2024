#include "ActionManager.h"

ActionManager::ActionManager(Communication * comESP, Communication * comArduino , bool test){
    
    MoveProfilesSetup::setup();
    gameTable.setup(couleurRobot);


    this->comESP=comESP;
    this->comArduino=comArduino;
    this->activeTestEnvironment=test;
    if (activeTestEnvironment){
        robot = new Robot(0.4,0.4,0.0);
    }
    else{
        if(couleurRobot == Vert){
            robot = new Robot(0.225,0.225,PI/2); //vert
        }else{
            robot = new Robot(2.0-0.225,0.225,PI/2); //bleu ou jaune, personne ne sait
            
        }
            
    }
}

bool ActionManager::getEnnemy(){
    return ennemy;
}

void ActionManager::setEnnemy(bool e){
    this->ennemy = e;
}

void ActionManager::setEnnemyDistance(float eDist){
    this->ennemyDistance = eDist;
}

void ActionManager::setEnnemyAngle(float eAngle){
    this->ennemyAngle = eAngle;
}

void ActionManager::update(float dt){
    //Serial.println(idTaskInProgress);
    switch(idTaskInProgress){
        case Strategie : //choix strategie
            updateStrategie();
            //TODO : Fonction de strategie qui renvoie la destination - l'action a realiser, etc...
            break;
        case CherchePalet : //recuperer les palets
            //Serial.println("bbbb");
            updateTaskCherchePalet();
            break;
        case DeposePalet : //deposer les palets
            updateTaskDepotGateau();
            break;
        case RecallageRotation :
            updateRecallageRotation();
            break;
        case RetourBase:
            updateRetourBase();
        case TestMouvement : 
            updateTestMouvement();
        default :
            break;
    }
}

void ActionManager::updateCommunicationMega(){
    if(comArduino->waitingRX()){
        waitMessageArduino = false;
        communicationArduinoEnd = true;
        comArduino->popOldestMessage();
    }
}



void ActionManager::stopMovement(){
    robot->stopMovement();
}

void ActionManager::updateStrategie(){
    if(activeTestEnvironment){
        idTaskInProgress=TestMouvement;
        nextDest=robot->kineticCurrent;
    }
    else{
        if (!stockageJaune&&!stockageRose&&!stockageMarron){//ATTENTION

            Vector nextCoord = gameTable.nearestPalet(robot->kineticCurrent,!stockageJaune,!stockageMarron,!stockageRose);
            float angle = (nextCoord-robot->kineticCurrent).angle();
            float angleNormalized = normalizeAngle(robot->kineticCurrent.getTheta()-angle);

            if(angleNormalized>PI/2||angleNormalized<-PI/2){
                idTaskInProgress=RecallageRotation;
                nextDest = VectorOriented(0,0,angle);
            }
            
            else{
                nextDest = VectorOriented(nextCoord.getX(), nextCoord.getY(),angle);
                idTaskInProgress=CherchePalet;
        //nextDest.printDebug("Next :");
          }
        }

        else{
            VectorOriented nextCoord = gameTable.nearestBase(robot->kineticCurrent);
            float angle = (nextCoord-robot->kineticCurrent).angle();
            float angleNormalized = normalizeAngle(robot->kineticCurrent.getTheta()-angle);
            nextDest.printDebug("Base :");

            if(angleNormalized>PI/2||angleNormalized<-PI/2){
                nextDest=VectorOriented(0,0,angle);
                idTaskInProgress=RecallageRotation;
            }
            else{
                nextDest=nextCoord;
                idTaskInProgress=DeposePalet;
        //nextDest.printDebug("Next :");
            }
        }
    }

   
}

void ActionManager::updateTaskCherchePalet(){
    //nextDest.printDebug("next");
    switch (idActionInCherchePalet)
    {
        case GoToPalet: //Mouvement au palet selectionne
            updateMoveAction(false,false);
            if(moveEnd){
                gameTable.getPaletInProgress()->onTable=false;
                //Serial.println(gameTable.getPaletInProgress()->onTable);
                idActionInCherchePalet=CommunicationArduinoCherche;
                actionInProgress=false;
                moveEnd=false;
            }
        break;

        case CommunicationArduinoCherche: //Update de la communication actionneurs en attendant la fin de l'action
            //Serial.println("envoi messg mega");
            envoiMessageMega();
            if(communicationArduinoEnd){
                Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!");
                idActionInCherchePalet=FinAction;
                communicationArduinoEnd=false;
                actionInProgress=false;
            }
        break;

        case FinAction: //Remise des parametres a zero et prise du palet 
                //Serial.println("JEEEEEEEEEEEEEEE");
            idTaskInProgress=Strategie;
            idActionInCherchePalet=GoToPalet;
            remplirStockage(gameTable.getCouleurPalet());
        break;

        default:
            break;
    }
}

void ActionManager::updateTaskDepotGateau(){

    switch(idActionInDeposeGateau){

        case GoToBase:{ //Deplacement a la base
            updateMoveAction(false,false);
            if(moveEnd){
                idActionInDeposeGateau=CommunicationArduinoDepot;
                actionInProgress=false;
                moveEnd=false;
            }
        break;}

        case CommunicationArduinoDepot:{ //envoi et attente du message de fin d'action de l'arduino
            envoiMessageMega();
            if(communicationArduinoEnd){
                idActionInDeposeGateau=StrategieSuiteGateauOuStop;
                communicationArduinoEnd=false;
                actionInProgress=false;
            }
        break;}

        case StrategieSuiteGateauOuStop :{ //choix si encore gateau a construire ou si fin donc choix de quel recul
            //countGateauinProgress+=1; 
            gameTable.getBaseInProgress()->setNbGateau(gameTable.getBaseInProgress()->getNbGateau()+1);
            float thetaBase = gameTable.getBaseInProgress()->getCoordonnees().getTheta();
            /*if(countGateauinProgress==3){*/
                nextDest=robot->kineticCurrent-VectorOriented(0.25*cos(thetaBase),0.25*sin(thetaBase),0.0);
                idActionInDeposeGateau=ReculFinalEtFin;
            //}//ATTENTION
            /*else{
                robot->kineticCurrent.printDebug("Robot");
                nextDest=robot->kineticCurrent-VectorOriented(0.15*cos(thetaBase),0.15*sin(thetaBase),0.0);
                nextDest.printDebug("nextDest");
                idActionInDeposeGateau=ReculEntreGateau;
            }*/
        break;}

        case ReculEntreGateau:{ //Recul entrainant un nouveau gateau
            updateMoveAction(false,true);
            if(moveEnd){
                idActionInDeposeGateau=StrategieSuiteGateauOuStop;
                actionInProgress=false;
                moveEnd=false;
            }
        break;}

        case ReculFinalEtFin :{ //Recul et reinitialisation des parametres pour la suite
            updateMoveAction(false,true);
            if(moveEnd){

                idActionInDeposeGateau=GoToBase;
                idTaskInProgress=Strategie;

                countGateauinProgress=0;

                stockageJaune=false;
                stockageMarron=false;
                stockageRose=false;

                actionInProgress=false;
                moveEnd=false;
            }
        break;}

        default :
            break;
    }
}

void ActionManager::updateRecallageRotation(){
    updateMoveAction(true,false);
    if (moveEnd){
        idTaskInProgress=Strategie;
        actionInProgress=false;    
        moveEnd=false;
    }
}


void ActionManager::updateMoveAction(bool isOnlyRotation, bool isBackward){
    if (actionInProgress){
        if (robot->endMovement()){
            moveEnd=true;   
        }
        else {
            robot->updateMovement();
        }
    }
    else{
        robot->startMovement(nextDest,isOnlyRotation,isBackward);
        backwardEvitement=!isBackward;
        actionInProgress=true;
    }   
}

void ActionManager::stopAction(){
    idTaskInProgress=Strategie;
    idActionInCherchePalet=StopCherche;
    idActionInDeposeGateau=StopDepot;

    
    waitMessageArduino = false;
    moveEnd=false;
    messageSend = false;
    actionInProgress=false;
}

bool ActionManager::robotInMovement(){
    return robot->getInMove();
}

void ActionManager::evitement(){
    if(!startEvitement){
        /*float diffAngle = (robot->kineticCurrent.getTheta() - ennemyAngle);
        normalizeAngle(diffAngle);*/
        if(ennemyAngle>PI/4||ennemyAngle<-PI/4){
            Serial.println("Hors-champ");
            ennemy=false;
        }
        else{
            stopMovement();
            stopAction();
            //nextDest=calculVectorOrientedEvitement();
            //Serial.println(backwardEvitement);
            //updateMoveAction(false,backwardEvitement);
            startEvitement=true;
            //robot->resumeMotor();
        }
    }
    else{
        //Serial.println("Evitement");
        updateMoveAction(false, backwardEvitement);
        if(moveEnd){
            ennemy=false;
            moveEnd=false;
            startEvitement=false;
            actionInProgress=false;
            idActionInCherchePalet=GoToPalet;
            idActionInDeposeGateau=GoToBase;

        }
    }
}

VectorOriented ActionManager::calculVectorOrientedEvitement(){
    float currentAngle = robot->kineticCurrent.getTheta();
    VectorOriented nextDestination = robot->kineticCurrent - VectorOriented((0.1-ennemyDistance)*cos(currentAngle),(0.1-ennemyDistance)*sin(currentAngle),0);
    //nextDestination.printDebug("next");
    //robot->kineticCurrent.printDebug("Robot");
    return nextDestination;
}

void ActionManager::remplirStockage(CouleurPalet coul){
    switch(coul)
        {
            case Jaune :
                stockageJaune=true;
                break;
            case Marron :
                stockageMarron=true;
                break;
            case Rose :
                stockageRose=true;
                break;
            default :
                break;
        }
}

void ActionManager::envoiMessageMega(){
    
    
        if(actionInProgress){
            if (waitMessageArduino){
                updateCommunicationMega();
                /*waitMessageArduino=false; //ATTENTION A CHANGER
                communicationArduinoEnd=true;*/

                
            }
            else{
                Serial.println("Fin Action");
                actionInProgress=false;
                
            }
        } 

        else{
            Message mess;
            if(idActionInCherchePalet==CommunicationArduinoCherche){
                switch(gameTable.getCouleurPalet()){
                    case Jaune :
                        mess = newMessageToDo(Teensy,Arduino,PaletJaune);
                    break;

                    case Marron :
                        mess = newMessageToDo(Teensy,Arduino,PaletMarron);
                    break;

                    case Rose :
                        mess = newMessageToDo(Teensy,Arduino,PaletRose);
                    break;

                    default :
                    break;

                }
            }
            else if(idActionInDeposeGateau==CommunicationArduinoDepot){
                mess = newMessageToDo(Teensy,Arduino,Depot);
            }
            Serial.println("SendMega");
            comArduino->send(mess);
            waitMessageArduino = true;
            actionInProgress=true;
            
        }
}

void ActionManager::updateRetourBase(){
    if(!startRetourBase&&!startRotationRetourBase){
            stopMovement();
            stopAction();
            robot->odometry.updateOdometry(0.01);
            delay(300);
            robot->odometry.updateOdometry(0.3);

            nextDest=gameTable.nearestBaseEmpty(robot->kineticCurrent);

            updateMoveAction(true,false);
            startRotationRetourBase=true;
            robot->resumeMotor();
        }
    else if (!startRetourBase&&startRotationRetourBase){
        updateMoveAction(true,false);
        if (moveEnd){
            startRetourBase=true;
            actionInProgress=false;
            moveEnd=false;
        }
    }
    else{
        updateMoveAction(false,false);
        if(moveEnd){
            ennemy=false;
            moveEnd=false;
            startRetourBase=false;
            actionInProgress=false;
            idActionInCherchePalet=StopCherche;
            idActionInDeposeGateau=StopDepot;
        }
    }

}


void ActionManager::updateTestMouvement(){
    switch(idActionTest){
        /*case 0 :{
            if(test2direc){
                float angle = nextDest.getTheta();
                nextDest=nextDest-VectorOriented(-0.855*cos(angle),0.0,0.0);
            }
            else{
                float angle = nextDest.getTheta();
                nextDest=nextDest-VectorOriented(0.0,-1.85*sin(angle),0.0);
            }
            idActionTest=1;
        break;}
        case 1 :
            updateMoveAction(false,false);
            if(moveEnd){
                    idActionTest=2;
                    test2direc=!test2direc;
                    actionInProgress=false;
                    moveEnd=false;


            }
        break;
        case 2 :
            nextDest = nextDest - VectorOriented(0.0,0.0,-PI/2);
            idActionTest=3;
        break;
        case 3 :
            updateMoveAction(true,false);
            if(moveEnd){
                idActionTest=0;
                actionInProgress=false;
                moveEnd=false;
            }
        break;*/
        /*case 0 :{
            nextDest=VectorOriented(1.9,0.0,0.0);
            idActionTest=1;
        break;}
        case 1 :
            updateMoveAction(false,false);
            if(moveEnd){
                    actionInProgress=false;
                    moveEnd=false;
                    stopMovement();
            }
        break;*/
        case 0 :{
            nextDest=nextDest - VectorOriented(0.0,0.0,-2*PI/3);
            idActionTest=1;
        break;}
        case 1 :
            updateMoveAction(true,false);
            if(moveEnd){
                counBoucleTest=0;
                if (counBoucleTest>29){
                    actionInProgress=false;
                    moveEnd=false;
                    stopMovement();
                }
                else{
                    
                    actionInProgress=false;
                    moveEnd=false;
                    idActionTest=0;
                }
            }
        break;
        default:
            break;
    }
}



/*void ActionManager::updateCommunicationESP(){
    if(comESP->waitingRX()){
        Message currentMessage = comESP->peekOldestMessage();
        DataID did = currentMessage.did;
        switch(did){

            case Coordonnees : {
                Serial.println("zzzzzzzzzz");
                VectorOriented next = VectorOriented(currentMessage.x/1000.0,currentMessage.y/1000.0);
                nextDest=next;
                setWaitMessage(false);
                break;
            }

            case MessLidar : {
                ennemy = true;
            }
        default:
            break;
    
        }
        comESP->popOldestMessage();
    }
}*/