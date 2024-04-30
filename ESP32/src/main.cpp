#include <Arduino.h>
#include "Communication.h"
#include "Message.h"
#include "IDS.h"

#include "YDLidarG4.h"
#include "WiFi.h"

#include "LiquidCrystal_I2C.h"

#include "UltrasonicSensor.h"

#include <FastLED.h>

#define LED_PIN     18
#define NUM_LEDS    41
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

#define UPDATES_PER_SECOND 100
CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType    currentBlending;



LiquidCrystal_I2C lcd(0x27, 16, 2);  //16 x 2 characters

#define LIDAR_ENABLE true

// Angles et distances des zones d'évitement
#define ANGLE_ZONE_0 45
#define DISTANCE_ZONE_0 500 //distance définie a partir du centre du robot (centre du LIDAR)

#define PIN_TRIG1 12
#define PIN_ECHO1 13

#define PIN_TRIG2 14
#define PIN_ECHO2 15

int distance1;
int distance2;

YDLidarG4 lidar;

Communication comTeensy = Communication(&Serial);


ultrasonicSensor sensor1 = ultrasonicSensor(PIN_TRIG1, PIN_ECHO1);
ultrasonicSensor sensor2 = ultrasonicSensor(PIN_TRIG2, PIN_ECHO2);


void updateCommunicationTeensy(){

}

void setup() {
  lcd.init();
  lcd.backlight();
  delay(100);
  lcd.clear();
  lcd.setCursor(0,0);   
  lcd.print("---ENSMASTEEL---");

  Serial.begin(115200);

   FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  
  
}





int lastDetection = millis();
bool was_none = true;

int k = 0;


void white(){

  for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(255,255,255);
  }

  FastLED.show();

}

void red(){

  for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(255,0,0);
  }

  FastLED.show();

}

void orange(){

  for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(255,130,0);
  }

  FastLED.show();

}


void loop() {

  comTeensy.update();
    
  if ((millis() - lastDetection) > 500 && !was_none){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("---ENSMASTEEL---");
    was_none = true;
    white();
  }

  distance1 =  sensor1.distance();
  distance2 = sensor2.distance();

  


  if((distance1 > 5 && distance1 < 40 )|| (distance2 > 5 && distance2 < 40)){
    lastDetection = millis();
    lcd.setCursor(0,1);
    lcd.print(distance1);
    lcd.print("  ");
    lcd.print(distance2);
    k ++;
    orange();
  }else{
    k = 0;
  }

 
 
  if(was_none && ((millis() - lastDetection) < 500) && k >= 3){
    Message mess = newMessageLidar(ESP_32, Teensy, 0, 0);
    comTeensy.send(mess);
    lcd.clear();
    lcd.print("Detection");
    was_none = false;
    red();
  }

}
