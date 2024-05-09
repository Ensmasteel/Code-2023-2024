#include <Arduino.h>
#include "SequenceManager.h"
#include "Sequence.h"
#include "Action.h"
#include "Robot.h"
#include "Asservissement.h"
#include "Message.h"
#include "TeensyThreads.h"
#include "Vector.h"

#define dt 0.01
#define MS_WAIT_ON_ENEMY 300

Robot* robot;
SequenceManager* brain;

Threads::Mutex tirrette_mut;

void threadOdometry() {
    while (1) {
        while (!tirrette_mut.getState()) {
            robot->updateOdometry(1);
            threads.delay(1);
        }
        threads.yield();
    }
}

void threadCommunications() {
    while (1) {
        while (!tirrette_mut.getState()) {
            robot->comMega.update();
            robot->comESP.update();
            threads.yield();
        }
    }
}

void threadSequence() {
    while (1) {
        while (!tirrette_mut.getState()) {
            brain->update(dt, robot);
            robot->getGhost().getCurVectO().printTeleplot("GHOST ");
            robot->getCurKinetic().printTeleplot("ROBOT ");
            threads.delay(1000 * dt);
        }
        threads.yield();
    }
}

void threadReceiveMsgESP() {
    unsigned int msLastMsg = 0;
    Message currentMessage;
    while (1) {
        if (robot->comESP.waitingRX()) {
            currentMessage = robot->comESP.peekOldestMessage();
            switch (currentMessage.did) {
                case MessLidar:
                    Logger::teleplot("> LIDAR xy :" + String(currentMessage.distance / 1000.0f * std::cos(currentMessage.angle / 1000.0f),3) + ":" + String(currentMessage.distance / 1000.0f * std::sin(currentMessage.angle / 1000.0f),3) + "|xy");
                    brain->setEnemy(true, currentMessage.distance / 1000.0f, currentMessage.angle / 1000.0f);
                    break;
                default:
                    break;
            }
            msLastMsg = millis();
            robot->comESP.popOldestMessage();
        }
        if (brain->getEnemy() && (millis() - msLastMsg) > MS_WAIT_ON_ENEMY) {
            // the enemy has not been detected for MS_WAIT_ON_ENEMY ms, we forget it
            brain->setEnemy(false);
            brain->resume();
            robot->resumeMotor();
            robot->getGhost().setLock(false);
        }
        threads.yield();
    }
}

void threadArretUrgence() {
    bool stop = false;
    while (1) {
        if (!stop && digitalRead(PIN_ARRET_URGENCE) == LOW) {
            robot->stopMovement();
            threads.stop();
            stop = true;
        }
        if (stop && digitalRead(PIN_ARRET_URGENCE) == HIGH) {
            SRC_GPR5 = 0x0BAD00F1;
            SCB_AIRCR = 0x05FA0004;
        }
        threads.yield();
    }
}

void threadTirette() {
    while (tirrette_mut.getState()) {
        if (robot->testTirette()) {
            tirrette_mut.unlock();
            threads.kill(threads.id());
        }
        threads.yield();
    }
}

void threadEnd() {
    bool first = true;
    float startMillis;
    float actMillis;
    while (1) {
        while (!tirrette_mut.getState()) {
            if (first) {
                startMillis = millis();
                first = false;
            } else {
                actMillis = millis() - startMillis;
                if ((actMillis >= 90000 && actMillis < 99000)) {
                    threads.suspend(2);
                    brain->forceRetourBase();
                    threads.delay(1000 * dt);
                }
                if (actMillis >= 99000) {
                    robot->stopMovement();
                    threads.suspend(2);  // STOP ACTION + EVITEMENT SEULEMENT ATTENTION A L'ORDRE DES ADD THREADS
                    threads.suspend(5);
                    tirrette_mut.lock();
                    threads.stop();
                }
            }
            threads.yield();
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(115200);

    if (CrashReport) {
        while(!Serial){
            delay(1000);
            Serial.print(CrashReport);
            delay(5000);
        }
    }

    /* SEQUENCES */
    robot = new Robot(0.0f, 1.5f, 0.0f);
    //  Sequences d'Attente  //
    Sequence Attendre(
        {
            new DelayAction(1000)
        }
    );
    // Sequences de pinces  //
    Sequence Prendre_Pots(
        {
            new StaticAction(CLOSE_CLAWS),
            new StaticAction(RAISE_CLAWS,true)
        }
    );
    Sequence Poser_Pots(
        {
            new StaticAction(LOWER_CLAWS),
            new StaticAction(OPEN_CLAWS)
        }
    );
    Sequence Lacher_Pots(
        {
            new StaticAction(OPEN_CLAWS)
        }
    );
    Sequence Descendre_Pince(
        {
            new StaticAction(LOWER_CLAWS)
        }
    );

    //  Sequences de déplacement  //
    Sequence Retour_Base(
        {
            new MoveAction(VectorOriented(0.0f, 0.0f, PI), false, false, true, true)
        }
    );
    Sequence Sortie_Base(
        {
            new MoveAction(VectorOriented(0.8f, 0.0f, 0.0f), false, false, true, true)
        }
    );
    Sequence arriere(
        {
            new MoveAction(VectorOriented(0.f, 0.0f, 0.0f), false, true, true, true),
        }
    );
    Sequence test_aller_retour(
        {
            new MoveAction(VectorOriented(1.0f, 0.0f, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0.0f), false, true, true, true),
        }
    );
    Sequence test_spin(
        {
            new MoveAction(VectorOriented(0.0f, 0.0f, PI), true, false, true, true),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0.0f), true, false, true, true),
        }
    );
    /* 
    Sequence Brain_Bleu(
        {
            new MoveAction(VectorOriented(0.45f, 0.45f, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0.0f), false, true, true, true),
        }
    );
    */
    /* 
    Sequence Brain_Jaune(
        {
            new MoveAction(VectorOriented(1.0f, 0.0f, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0.0f), false, true, true, true),
        }
    );
    */
    //robot = new Robot(0.0f, 0.0f, 0.0f);
    Sequence Test_premier_triangle(
        {
            new MoveAction(VectorOriented(1.0f, 0.85f, 0.7045f), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            new StaticAction(RAISE_CLAWS,true),
            new MoveAction(VectorOriented(0.45f, 0.6125f, 0.0), false, true, true, true),
            new StaticAction(START_MAGNET,true),
            new MoveAction(VectorOriented(0.065f, 0.6125f, 0.0), true, false, true, true),
            new MoveAction(VectorOriented(0.1f, 0.6125f, 0.0), false, false, true, true),
            new MoveAction(VectorOriented(0.1f, 0.6125f, PI/2), false, true, true, true),
            new MoveAction(VectorOriented(0.1f, 0.065f, PI/2), true, false, true, true),
            new StaticAction(SHUTDOWN_MAGNET,true),
            new MoveAction(VectorOriented(0.1f, 0.1f, PI/2), false, false, true, true),
            new MoveAction(VectorOriented(0.1f, 0.1f, -PI/2), false, true, true, true),
            new MoveAction(VectorOriented(0.1f, 0.065f, -PI/2), false, false, true, true),
            new StaticAction(OPEN_CLAWS),
            new MoveAction(VectorOriented(0.1f, 0.1f, -PI/2), true, false, true, true),
            new StaticAction(LOWER_CLAWS)
        }
    );
    Sequence Rush(
        {
            new MoveAction(VectorOriented(1.5f, 0.0f, 0.0f), false, false, true, true)
        }
    );
    //robot = new Robot(0.0f, 2.0f, 0.0f);
    Sequence Moins_Rush_Bleu(
        {
            new MoveAction(VectorOriented(0.5f, 1.5-0.4f, -0.7045f), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            new MoveAction(VectorOriented(0.5f, 1.5-0.4f, -PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.5f, 0.05, 0.0f), false, false, true, true),
            new StaticAction(SOLAR_RIGHT_ON),
            new MoveAction(VectorOriented(0.0f, 0.05, 0.0f), false, true, true, true),
            new StaticAction(OPEN_CLAWS,true),
            new StaticAction(SOLAR_RIGHT_OFF)
        }
    );
    Sequence Moins_Rush_Jaune(
        {
            new MoveAction(VectorOriented(0.5f, 0.40f, 0.7045f), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            new MoveAction(VectorOriented(0.5f, 0.40f, PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.5f, 1.45f, PI), false, false, true, true),
            new StaticAction(SOLAR_RIGHT_ON),
            new MoveAction(VectorOriented(0.0f, 1.45f, PI), false, false, true, true),
            new StaticAction(OPEN_CLAWS,true),
            new StaticAction(SOLAR_RIGHT_OFF)
        }
    );
    Sequence Test_Solaires(
        {
            new StaticAction(SOLAR_RIGHT_ON,true),
            new MoveAction(VectorOriented(1.0f, 0.0f, 0.0f), false, false, true, true)
        }
    );
    Sequence Carre(
        {
            new MoveAction(VectorOriented(0.6f, 0.0f, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.6f, 0.0f, -PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.6f, -0.6f, -PI/2), false, false, true, true),
            new MoveAction(VectorOriented(0.6f, -0.6f, PI), true, false, true, true),
            new MoveAction(VectorOriented(0.0f, -0.6f, PI), false, false, true, true),
            new MoveAction(VectorOriented(0.0f, -0.6f, PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.0f, 0.0f, PI/2), false, false, true, true),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0.0f), true, false, true, true)
        }
    );
    Sequence bras(
        {
            new StaticAction(START_MAGNET,true),
            new StaticAction(SOLAR_RIGHT_ON,true),
            new StaticAction(SOLAR_LEFT_ON),
            new DelayAction(3000),
            new StaticAction(SOLAR_RIGHT_OFF,true),
            new StaticAction(SOLAR_LEFT_OFF,true),
            new StaticAction(SHUTDOWN_MAGNET)
        }
    );
    brain = new SequenceManager({bras}); 

    /* MISC */
    MoveProfilesSetup::setup();
    threads.setMicroTimer(10);
    threads.setDefaultTimeSlice(1);

    //tirrette_mut.lock();

    Logger::setup(&Serial, &Serial, &Serial, false, false, true);

    delay(3000);
    threads.addThread(threadEnd);
    threads.addThread(threadSequence);
    threads.addThread(threadArretUrgence);
    pinMode(PIN_ARRET_URGENCE, INPUT_PULLDOWN);
    threads.addThread(threadOdometry);
    threads.addThread(threadReceiveMsgESP);
    threads.addThread(threadCommunications);
    threads.addThread(threadTirette);

}

void loop() {}
