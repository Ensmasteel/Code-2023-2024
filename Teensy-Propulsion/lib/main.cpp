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
SequenceManager* brain_bleu;
SequenceManager* brain_jaune;

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
    Message msg;
    while (tirrette_mut.getState()) {
        if (robot->testTirette()) {
            tirrette_mut.unlock();
            threads.kill(threads.id());
        }

        /* Handle team color changes while the tirette is not released */
        robot->comMega.update();
        if (robot->comMega.waitingRX()) {
            msg = robot->comMega.peekOldestMessage();
            switch (msg.did) {
                case MessActuator:
                    switch (msg.aid){
                    case SetTeamColorJaune:
                        brain = brain_jaune;
                        robot->comESP.send(newMessageActuator(Teensy, ESP_32, SetTeamColorJaune));
                        break;
                    case SetTeamColorBleu:
                        brain = brain_bleu;
                        robot->comESP.send(newMessageActuator(Teensy, ESP_32, SetTeamColorBleu));
                        break;
                    default:
                        break;
                    }
                    break;
                default:
                    break;
            }
            robot->comMega.popOldestMessage();
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

    Sequence Brain_Bleu(
        {
            new StaticAction(SOLAR_RIGHT_ON),
            new MoveAction(VectorOriented(0.635f, 0.0f, 0.0f), false, false, true, true),
            new StaticAction(SOLAR_RIGHT_OFF),
            new MoveAction(VectorOriented(0.635f, 0.0f, PI), true, false, true, true),
            new StaticAction(START_MAGNET,true),
            new MoveAction(VectorOriented(0.650, 0.0f, PI), false, true, true, true),
            new MoveAction(VectorOriented(0.650, 0.0f, PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.650, 0.835, PI/2), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            new StaticAction(RAISE_CLAWS,true),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0.0), false, true, true, true),
            new StaticAction(SHUTDOWN_MAGNET,true),
            new MoveAction(VectorOriented(0.15, 0.0f, 0.0), false, false, true, true),
            new MoveAction(VectorOriented(0.15, 0.0f, PI), true, false, true, true),
            new MoveAction(VectorOriented(0.15, 0.0f, PI), false, false, true, true),
            new StaticAction(OPEN_CLAWS),
            new MoveAction(VectorOriented(0.15, 0.0f, 0.0), false, true, true, true),
            new StaticAction(LOWER_CLAWS)
        }
    );
    Sequence Brain_Jaune(
        {
            new StaticAction(SOLAR_LEFT_ON),
            new MoveAction(VectorOriented(0.70f, 1.77, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.70f, 1.77, -0.9*PI/4), true, false, true, true),
            new StaticAction(SOLAR_LEFT_OFF,true),
            new MoveAction(VectorOriented(1.0f, 1.4f, -PI/2), false, false, true, true),
            new MoveAction(VectorOriented(0.85f, 1.2f, -3*PI/4), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            new MoveAction(VectorOriented(0.94f, 1.69f, -PI/2), false, true, true, true),
            new StaticAction(START_MAGNET,true),
            new MoveAction(VectorOriented(0.94f, 1.71f, -PI/2), false, true, true, true),

            new MoveAction(VectorOriented(0.94f, 1.35f, -PI/2), false, false, true, true),

            new MoveAction(VectorOriented(1.11f, 1.73f, -PI/2), false, true, true, true),
            new MoveAction(VectorOriented(1.11f, 1.77f, -PI/2), false, true, true, true, 1500),
            new MoveAction(VectorOriented(1.23f, 1.47f, -PI/4), false, false, true, true),
            new MoveAction(VectorOriented(0.26f, 1.72f, 0.0f), false, true, true, true),
            new StaticAction(SHUTDOWN_MAGNET,true),
            new StaticAction(RAISE_CLAWS,true),
            new MoveAction(VectorOriented(0.45f, 1.72f, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.45f, 1.72f, PI), true, false, true, true),
            new MoveAction(VectorOriented(0.22f, 1.72f, PI), false, false, true, true),
            new StaticAction(OPEN_CLAWS)

            /*
            new StaticAction(SOLAR_LEFT_OFF),
            new MoveAction(VectorOriented(0.625f, 0.0f, -PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.625f, 0.2f, -PI/2), false, false, true, true),

            new MoveAction(VectorOriented(0.625f, 0.2f, -3*PI/4), true, false, true, true),

            new StaticAction(START_MAGNET,true),
            new MoveAction(VectorOriented(0.650, 0.0f, PI), false, true, true, true),
            new MoveAction(VectorOriented(0.650, 0.0f, -PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.650, -0.835, -PI/2), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            new StaticAction(RAISE_CLAWS,true),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0.0), false, true, true, true),
            new StaticAction(SHUTDOWN_MAGNET,true),
            new MoveAction(VectorOriented(0.15, 0.0f, 0.0), false, false, true, true),
            new MoveAction(VectorOriented(0.15, 0.0f, PI), true, false, true, true),
            new MoveAction(VectorOriented(0.15, 0.0f, PI), false, false, true, true),
            new StaticAction(OPEN_CLAWS),
            new MoveAction(VectorOriented(0.15, 0.0f, 0.0), false, true, true, true),
            new StaticAction(LOWER_CLAWS)
            */

        }
    );
        Sequence Brain_Test(
        {
            new StaticAction(SOLAR_LEFT_OFF),
            new StaticAction(SOLAR_RIGHT_OFF),
            new StaticAction(SOLAR_LEFT_ON),
            new DelayAction(3000),
            new StaticAction(SOLAR_LEFT_OFF)
        }
    );
    robot = new Robot(0.14f, 1.77f, 0.0f);
    brain_bleu = new SequenceManager({Brain_Jaune}); 
    brain_jaune = new SequenceManager({Brain_Jaune});
    // Default team color is blue
    brain = brain_bleu;
    robot->comESP.send(newMessageActuator(Teensy, ESP_32, SetTeamColorBleu));

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

    // TODO delete unused brain

}

void loop() {}
