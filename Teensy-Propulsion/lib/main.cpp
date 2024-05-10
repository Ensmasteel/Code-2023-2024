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
VectorOriented robotInitBleu(3.0f - 0.14f, 1.77f, PI);
VectorOriented robotInitJaune(0.14f, 1.77f, 0.0f);
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
                        robot->init(robotInitJaune.getX(), robotInitJaune.getY(), robotInitJaune.getTheta());
                        robot->comESP.send(newMessageActuator(Teensy, ESP_32, SetTeamColorJaune));
                        break;
                    case SetTeamColorBleu:
                        brain = brain_bleu;
                        robot->init(robotInitBleu.getX(), robotInitBleu.getY(), robotInitBleu.getTheta());
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
                // if ((actMillis >= 90000 && actMillis < 99000)) {
                //     threads.suspend(2);
                //     brain->forceRetourBase();
                //     threads.delay(1000 * dt);
                // }
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

float decalage = 0.08f;

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
            new MoveAction(VectorOriented(3.0f - 0.70f,  1.77, normalizeAngle(PI -  0.0f)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.70f,  1.77, normalizeAngle(PI -  -0.9*PI/4)), true, false, true, true),
            new StaticAction(SOLAR_RIGHT_OFF,true),
            new MoveAction(VectorOriented(3.0f - 0.91,  1.50f, normalizeAngle(PI -  -PI/4)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 1.3f,  1.25f, normalizeAngle(PI -  -0.0f)), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            new MoveAction(VectorOriented(3.0f - 0.94f,  1.69 + decalage, normalizeAngle(PI -  -PI/2)), false, true, true, true),
            new StaticAction(START_MAGNET,true),
            new MoveAction(VectorOriented(3.0f - 0.94f,  1.70 + decalage, normalizeAngle(PI -  -PI/2)), false, true, true, true),
            new MoveAction(VectorOriented(3.0f - 0.94f,  1.35f + decalage, normalizeAngle(PI -  -PI/2)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 1.11f,  1.73f + decalage, normalizeAngle(PI -  -PI/2)), false, true, true, true),
            new MoveAction(VectorOriented(3.0f - 1.11f,  1.78f + decalage, normalizeAngle(PI -  -PI/2)), false, true, true, true, 1000),
            new MoveAction(VectorOriented(3.0f - 1.23f,  1.47f, normalizeAngle(PI -  -PI/4)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.26f,  1.72f, normalizeAngle(PI -  0.0f)), false, true, true, true),
            new StaticAction(SHUTDOWN_MAGNET,true),
            new StaticAction(RAISE_CLAWS,true),
            new MoveAction(VectorOriented(3.0f - 0.45f,  1.72f, normalizeAngle(PI -  0.0f)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.45f,  1.72f, normalizeAngle(PI -  PI)), true, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.22f,  1.72f, normalizeAngle(PI -  PI)), false, false, true, true),
            new StaticAction(OPEN_CLAWS),
            new MoveAction(VectorOriented(3.0f - 0.42f,  1.72f, normalizeAngle(PI -  PI)), false, true, true, true),
            new StaticAction(LOWER_CLAWS,true),
            new MoveAction(VectorOriented(3.0f - 0.42f,  1.72f, normalizeAngle(PI -  -3*PI/8)), true, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.90f,  0.83f, normalizeAngle(PI -  -3*PI/8)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 1.12f,  0.75f, normalizeAngle(PI -  0.0f)), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            //  La suite des valeurs en X est affectée de -4 cm
            new MoveAction(VectorOriented(3.0f - 0.32f,  0.67f, normalizeAngle(PI -  0.0f)), false, true, true, true),
            new StaticAction(START_MAGNET,true),
            new MoveAction(VectorOriented(3.0f - 0.29f,  0.67f, normalizeAngle(PI -  0.0f)), false, true, true, true),
            new MoveAction(VectorOriented(3.0f - 0.62f,  0.5f, normalizeAngle(PI -  0.0f)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.32f,  0.5f, normalizeAngle(PI -  0.0f)), false, true, true, true),
            new MoveAction(VectorOriented(3.0f - 0.29f,  0.5f, normalizeAngle(PI -  0.0f)), false, true, true, true, 1000),
            new MoveAction(VectorOriented(3.0f - 0.33f,  0.5f, normalizeAngle(PI -  0.0f)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.33f,  0.5f, normalizeAngle(PI -  PI/2)), true, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.33f,  0.23f, normalizeAngle(PI -  PI/2)), false, true, true, true),
            new StaticAction(SHUTDOWN_MAGNET,true),
            new StaticAction(RAISE_CLAWS,true),
            new MoveAction(VectorOriented(3.0f - 0.33f,  0.36f, normalizeAngle(PI -  PI/2)), false, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.33f,  0.36f, normalizeAngle(PI -  -PI/2)), true, false, true, true),
            new MoveAction(VectorOriented(3.0f - 0.33f,  0.22f, normalizeAngle(PI -  -PI/2)), false, false, true, true),
            new StaticAction(OPEN_CLAWS),
            new MoveAction(VectorOriented(3.0f - 0.37f,  0.30f, normalizeAngle(PI -  -PI/2)), false, true, true, true)
        }
    );
    Sequence Brain_Jaune(
        {
            new StaticAction(SOLAR_LEFT_ON),
            new MoveAction(VectorOriented(0.70f, 1.77, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.70f, 1.77, -0.9*PI/4), true, false, true, true),
            new StaticAction(SOLAR_LEFT_OFF,true),
            new MoveAction(VectorOriented(1.0f, 1.4f, -PI/2), false, false, true, true),
            new MoveAction(VectorOriented(0.85f, 1.25f, -4*PI/5), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            new MoveAction(VectorOriented(0.94f, 1.69f, -PI/2), false, true, true, true),
            new StaticAction(START_MAGNET,true),
            new MoveAction(VectorOriented(0.94f, 1.71f, -PI/2), false, true, true, true),
            new MoveAction(VectorOriented(0.94f, 1.35f, -PI/2), false, false, true, true),
            new MoveAction(VectorOriented(1.11f, 1.73f, -PI/2), false, true, true, true),
            new MoveAction(VectorOriented(1.11f, 1.77f, -PI/2), false, true, true, true, 1000),
            new MoveAction(VectorOriented(1.18f, 1.47f, -PI/4), false, false, true, true),
            new MoveAction(VectorOriented(0.26f, 1.72f, 0.0f), false, true, true, true),
            new StaticAction(SHUTDOWN_MAGNET,true),
            new StaticAction(RAISE_CLAWS,true),
            new MoveAction(VectorOriented(0.45f, 1.72f, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.45f, 1.72f, PI), true, false, true, true),
            new MoveAction(VectorOriented(0.22f, 1.72f, PI), false, false, true, true),
            new StaticAction(OPEN_CLAWS),
            new MoveAction(VectorOriented(0.42f, 1.72f, PI), false, true, true, true),
            new StaticAction(LOWER_CLAWS,true),
            new MoveAction(VectorOriented(0.42f, 1.72f, -3*PI/8), true, false, true, true),
            new MoveAction(VectorOriented(0.90f, 0.78f, -3*PI/8), false, false, true, true),
            new MoveAction(VectorOriented(1.12f, 0.70f, 0.0f), false, false, true, true),
            new StaticAction(CLOSE_CLAWS),
            //  La suite des valeurs en X est affectée de -4 cm
            new MoveAction(VectorOriented(0.24f, 0.68f, 0.0f), false, true, true, true),
            new StaticAction(START_MAGNET,true),
            new MoveAction(VectorOriented(0.21f, 0.68f, 0.0f), false, true, true, true),  
            new MoveAction(VectorOriented(0.54f, 0.5f, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.16f, 0.5f, 0.0f), false, true, true, true),
            new MoveAction(VectorOriented(0.14f, 0.5f, 0.0f), false, true, true, true, 1000),
            new MoveAction(VectorOriented(0.21f, 0.5f, 0.0f), false, false, true, true),
            new MoveAction(VectorOriented(0.21f, 0.5f, PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.21f, 0.23f, PI/2), false, true, true, true),
            new StaticAction(SHUTDOWN_MAGNET,true),
            new StaticAction(RAISE_CLAWS,true),
            new MoveAction(VectorOriented(0.21f, 0.36f, PI/2), false, false, true, true),
            new MoveAction(VectorOriented(0.21f, 0.36f, -PI/2), true, false, true, true),
            new MoveAction(VectorOriented(0.21f, 0.22f, -PI/2), false, false, true, true),
            new StaticAction(OPEN_CLAWS),
            new MoveAction(VectorOriented(0.21f, 0.30f, -PI/2), false, true, true, true)
        }
    );

    brain_bleu = new SequenceManager({Brain_Bleu}); 
    brain_jaune = new SequenceManager({Brain_Jaune});
    // Default team color is blue
    robot = new Robot(robotInitBleu.getX(), robotInitBleu.getY(), robotInitBleu.getTheta());
    brain = brain_bleu;
    robot->comESP.send(newMessageActuator(Teensy, ESP_32, SetTeamColorBleu));

    /* MISC */
    MoveProfilesSetup::setup();
    threads.setMicroTimer(10);
    threads.setDefaultTimeSlice(1);

    tirrette_mut.lock();

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
