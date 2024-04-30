#include <Arduino.h>

#include "Servo.h"
#include "pinSetup.h"
#include "Communication.h"
#include "Logger.h"
//#include "Actuators.h"
//#include "Actuators_Manager.h"


Servo servoPinceGauche,servoPinceDroit;

Communication comTeensy = Communication(&Serial1);
Communication * ptrComTeensy;
bool open;
Message messStocke;
//ActionID aid;
//Actuator_Manager actuator_manager = Actuator_Manager(ptrComTeensy);

//Actuator_Manager actuator_manager = Actuator_Manager();




bool mess;

void updateCommunicationTeensy(){
  if (ptrComTeensy->waitingRX()){
    Serial.println("a");
    messStocke = ptrComTeensy->peekOldestMessage();
    mess=true;
  }
  ptrComTeensy->popOldestMessage();
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Logger::setup(&Serial,&Serial,true,true);
  servoPinceGauche.attach(PIN_LEFT_SERVO_PWM);
  servoPinceDroit.attach(PIN_RIGHT_SERVO_PWM);
  servoPinceGauche.write(0);
  servoPinceDroit.write(180);
  open=true;
  mess=false;
  ptrComTeensy=&comTeensy;

}

//ouvert 0, ferme 20 (gauche) ouvert 180, ferme 160 (droit)

void (loop)() {
  ptrComTeensy->update();
  updateCommunicationTeensy();
  if(mess){
    Serial1.println("11111111111");
    if(open){
        servoPinceGauche.write(25);
        servoPinceDroit.write(155);
    }
    else{
        servoPinceGauche.write(0);
        servoPinceDroit.write(180);
    }
    Message messSend = newMessageEndAction(Arduino,Teensy,PaletJaune);
    ptrComTeensy->send(messSend);
    open=!open;
    mess=false;
  }
}
/*
void setup(){
  delay(2000);
  actuator_manager.init();
}

void (loop)(){
aid = PaletMarron;
actuator_manager.setIdAction(aid);
actuator_manager.updateCommunication();

delay(3000);

aid = PaletJaune;

actuator_manager.setIdAction(aid);
actuator_manager.updateCommunication();
delay(3000);
aid = PaletRose;

actuator_manager.setIdAction(aid);
actuator_manager.updateCommunication();
delay(3000);
aid = Depot;

actuator_manager.setIdAction(aid);
actuator_manager.updateCommunication();
delay(3000);
}
*/