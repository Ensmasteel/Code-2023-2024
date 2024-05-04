#include <Arduino.h>
#include <FastLED.h>
#include "LiquidCrystal_I2C.h"
#include "Communication.h"
#include "Message.h"
#include "IDS.h"
#include "lidar.h"

#define LED_PIN     18
#define NUM_LEDS    41
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

Lidar lidar;
String lidar_msg;
char chr_tmp;
float distance, angle;

LiquidCrystal_I2C lcd(0x27, 16, 2);

Communication comTeensy = Communication(&Serial);
Message teen_msg;

void lidar_callback(const std::vector<PointData> & laser_data) {
    /* Angle resolution, the smaller the resolution, the smaller the error after conversion */
    //const float angle_increment = ANGLE_TO_RADIAN(lidar.GetSpeed() / 4500);

    for (const PointData& point : laser_data) {
        distance = point.distance / 1000.0;

        lcd.setCursor(2,1);
        lcd.print(distance);
        if (distance < 0.3) {
            teen_msg = newMessageLidar(ESP_32, Teensy, distance, ANGLE_TO_RADIAN(point.angle));
            comTeensy.send(teen_msg);
        }
        
    }
}

void setup() {
    Serial.begin(115200);

    Serial2.begin(230400);
    lidar.SetPopulateCallback(lidar_callback);

    lcd.init();
    lcd.backlight();
    delay(100);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   ENSMASTEEL   ");

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    for(int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(20, 2, 0);
    }
    FastLED.show();
}

void loop() {

    /* Update the Teensy communication */
    comTeensy.update();

    /* Handle the lidar messages */
    lidar_msg = "";
    chr_tmp = ' ';
    while (Serial2.available() && chr_tmp != '\n') {
        chr_tmp = (char)Serial2.read();
        lidar_msg += chr_tmp;
    }
    if (lidar_msg.length() > 0) {
        uint8_t lidar_bytes [lidar_msg.length()];
        lidar_msg.getBytes(lidar_bytes, lidar_msg.length());
        if (lidar.Parse(lidar_bytes, lidar_msg.length())) {
            lidar.AssemblePacket();
        }
    }

}
