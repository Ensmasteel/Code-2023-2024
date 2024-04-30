#ifndef ACTION_MANAGER_H_
#define ACTION_MANAGER_H_

#include "Arduino.h"
#include "Robot.h"
#include "Action.h"
#include "GameElement.h"

enum IDTask : uint8_t{
    Strategie,
    CherchePalet,
    DeposePalet,
    RecallageRotation,
    RetourBase,
    TestMouvement
};

enum IDActionInCherchePalet : uint8_t{
    GoToPalet,
    CommunicationArduinoCherche,
    FinAction,
    StopCherche
};

enum IDActionInDeposeGateau : uint8_t{
    GoToBase,
    CommunicationArduinoDepot,
    StrategieSuiteGateauOuStop,
    ReculEntreGateau,
    ReculFinalEtFin,
    StopDepot
};

class ActionManager{

    public :
        
        ActionManager(Communication * comESP, Communication * comArduino, bool test);

        bool getEnnemy();
        
        void setEnnemy(bool e);
        void setEnnemyAngle(float eAngle);
        void setEnnemyDistance(float eDist);

        void update(float dt);
        void updateCommunicationMega();
        void envoiMessageMega();

        void updateStrategie();
        void updateTaskCherchePalet();
        void updateTaskDepotGateau();
        void updateRecallageRotation();
        void updateRetourBase();
        void updateTestMouvement();

        void updateMoveAction(bool isOnlyRotation, bool isBackward);

        void remplirStockage(CouleurPalet coul);
        
        void evitement();
        VectorOriented calculVectorOrientedEvitement();
        void stopMovement();
        void stopAction();

        bool robotInMovement();
        Robot * robot;
        VectorOriented nextDest;
        GameElement gameTable;


    private :

        bool actionInProgress = false;
        bool waitMessageArduino = false;
        bool moveEnd=false;
        bool communicationArduinoEnd=false;
        bool messageSend = false;
        bool ennemy = false;

        
        bool stockageMarron = false;
        bool stockageRose = false;
        bool stockageJaune = false;

        IDTask idTaskInProgress = Strategie;
        IDActionInCherchePalet idActionInCherchePalet = GoToPalet;
        IDActionInDeposeGateau idActionInDeposeGateau = GoToBase;

        Communication * comESP;
        Communication * comArduino;

        

        CouleurBase couleurRobot = Bleu; //A changer

        uint8_t countGateauinProgress = 0;      

     
        float ennemyAngle = 0.0;
        uint16_t ennemyDistance = 0.0;

        bool startEvitement = false;
        bool backwardEvitement = false;

        bool startRetourBase = false;
        bool startRotationRetourBase = false;

        bool backwardInProgress = false;

        bool activeTestEnvironment;

        int idActionTest=0;
        int counBoucleTest=0;
        bool test2direc = true;
};


#endif