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

Robot* robot;
SequenceManager* brain;

Threads::Mutex tirrette_mut;

void threadOdometry() {
    while (1) {
        while (!tirrette_mut.getState()) {
            robot->updateOdometry(1.0 / 1000.0);
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
            threads.delay(1000 * dt);
            brain->update(dt, robot);
        }
        threads.yield();
    }
}

void threadReceiveMsgESP() {
    while (1) {
        if (robot->comESP.waitingRX()) {
            Message currentMessage = robot->comESP.peekOldestMessage();
            switch (currentMessage.did) {
                case MessLidar: {
                    Serial.print(currentMessage.distance);
                    Serial.print("    ");
                    Serial.println(currentMessage.angle);
                    brain->setEnemy(true, currentMessage.distance, currentMessage.angle);
                    break;
                }
                default:
                    break;
            }
            robot->comESP.popOldestMessage();
        }
        threads.delay(1000 * dt);
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
                    // threads.stop();
                }
            }
            threads.yield();
        }
    }
}

void setup() {
    /* SEQUENCES */
    robot = new Robot(0.0f, 0.0f, 0.0f);
    Sequence avancer_reculer(
        {
            new MoveAction(VectorOriented(0.3f, 0.0f, 0.0f), false, false),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0.0f), false, true)
        }
    );
    Sequence tourner(
        {
            new MoveAction(VectorOriented(0.0f, 0.0f, PI), true, false),
            new MoveAction(VectorOriented(0.0f, 0.0f, 0), true, false)
        }
    );
    brain = new SequenceManager({}, 0);

    /* MISC */
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(115200);
    MoveProfilesSetup::setup();
    threads.setMicroTimer(10);
    threads.setDefaultTimeSlice(1);
    // tirrette_mut.lock();
    Logger::setup(&Serial, &Serial, &Serial, false, false, false);
    delay(3000);
    threads.addThread(threadEnd);
    threads.addThread(threadSequence);
    threads.addThread(threadArretUrgence);
    pinMode(PIN_ARRET_URGENCE, INPUT_PULLDOWN);
    threads.addThread(threadOdometry);
    threads.addThread(threadReceiveMsgESP);
    threads.addThread(threadCommunications);
    threads.addThread(threadTirette);

    if (CrashReport) Serial.print(CrashReport);
}

void loop() {}
