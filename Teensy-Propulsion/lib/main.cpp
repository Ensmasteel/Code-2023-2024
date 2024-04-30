#include <Arduino.h>
#include <Logger.h>
#include "Vector.h"
#include "Math_functions.h"
#include "Ghost.h"
#include "Motor.h"
#include "ActionManager.h"

#include "Communication.h"
#include "Message.h"
#include "pinSetup.h"
#include "Tirette.h"

#include "TeensyThreads.h"

#define FREQUENCY 100
#define dt 1.0/FREQUENCY

bool start=false;

Communication comMega = Communication(&Serial1);
Communication comESP = Communication(&Serial2);

bool testEnv = false;

ActionManager BBB = ActionManager(&comESP,&comMega,testEnv);

Threads::Mutex mut;

Tirette tirette = Tirette(PIN_TIRETTE);

bool first ;
bool stop ;

float startMillis;
float actMillis;



void updateCommunicationEvitement(){
  //Serial.println();
    if(comESP.waitingRX()){
        Serial.println("Message");
        Message currentMessage = comESP.peekOldestMessage();
        switch(currentMessage.did){
          case MessLidar:{
            BBB.setEnnemy(true);
            BBB.setEnnemyDistance(currentMessage.distance/1000.0);
            BBB.setEnnemyAngle(normalizeAngle((currentMessage.angle)*2*PI/360.0));
            break;
          }
          default:
            break;
        }
        comESP.popOldestMessage();
    }
  }

void threadOdometry(){
  while(1){
    while(!mut.getState()){
      BBB.robot->odometry.updateOdometry(1/1000.0);
      threads.delay(1);
    }
    threads.yield();
  }
}

void threadCommunication(){
  while(1){
    while(!mut.getState()){
      comMega.update();
      comESP.update();
      threads.yield();
    }
  }
}


int delaythreadqsfd = 0;

void threadAction(){
  while(1){
    while(!mut.getState() && !BBB.getEnnemy()){
      threads.delay(1000 * dt);
      BBB.update(dt);
      //BBB.robot->kineticCurrent.printDebug("Robot");
      //BBB.robot->ghost.getControllerKinetic().printDebug("Ghost");
      Serial.println("frequency : " + String((float) 1000.0/(millis() - delaythreadqsfd)));
      
      delaythreadqsfd = millis();
    }
  
    threads.yield();

  
  }
}

void threadEvitement(){
  while(1){
    
    updateCommunicationEvitement();
    if (!mut.getState() && BBB.getEnnemy()){
      if(BBB.robotInMovement()){
        BBB.evitement();
        Serial.println("11111111111111");
      }
      else{
        Serial.println("1858517512");
        comESP={&Serial2};
      }
    }  
    
    threads.delay(1000 * dt);
  }
}

void threadArretUrgence(){
  while(1){
    if (!stop && digitalRead(PIN_ARRET_URGENCE)==LOW){
      BBB.robot->stopMovement();
      /*Message messStop = newMessageEndAction(Teensy,Arduino,PaletJaune);
      comMega.send(messStop);
      comMega.update();*/
      threads.stop();
      stop = true;
    }
    if(stop && digitalRead(PIN_ARRET_URGENCE)==HIGH){
      
      SRC_GPR5 = 0x0BAD00F1;
      SCB_AIRCR = 0x05FA0004;
    }
    threads.yield();
  }
}

void threadTirette(){
  while(mut.getState()){
    if (tirette.testTirette()){
      mut.unlock();
      threads.kill(threads.id());
    }
  threads.yield();
  }
}

void threadEnd(){
  while(1){
    while(!mut.getState()){
      if (first){
        Serial.println("Starttttttttttt");
        startMillis=millis();
        first=false;
      }
      else{
        actMillis=millis()-startMillis;
        if ((actMillis>=90000 && actMillis<99000) || BBB.gameTable.countPaletsInTable()==0){
          threads.suspend(2);;
          /*
          BBB.robot->stopMovement();
          delay(1000);
          BBB.robot->odometry.updateOdometry(0.01);
          */

          BBB.updateRetourBase();
          threads.delay(1000*dt);
        }
        if(actMillis>=99000){
          BBB.robot->stopMovement();
          threads.suspend(2); //STOP ACTION + EVITEMENT SEULEMENT ATTENTION A L'ORDRE DES ADD THREADS
          threads.suspend(5);
          //threads.stop();
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
  
  threads.setMicroTimer(10);
  threads.setDefaultTimeSlice(1);
  first=true;
  stop=false;
  mut.lock();
  
  
  Logger::setup(&Serial, &Serial, true, true  );
  delay(3000);
  //mut.unlock();
  threads.addThread(threadEnd);
  threads.addThread(threadAction);
  threads.addThread(threadArretUrgence);
  pinMode(PIN_ARRET_URGENCE,INPUT_PULLDOWN);
  threads.addThread(threadOdometry);
  threads.addThread(threadEvitement);
  threads.addThread(threadCommunication);
  threads.addThread(threadTirette);
  
  //mut.lock();

}

void loop() {

}