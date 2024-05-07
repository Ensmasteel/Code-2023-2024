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

LiquidCrystal_I2C lcd(0x27, 16, 2);

Communication comTeensy = Communication(&Serial);
Message teen_msg;

Lidar lidar(&Serial2);
lidarFrame lidar_frame;
float angle_step;
uint16_t distance;
float angle;
unsigned int point_id;

void setup() {
    Serial.begin(115200);
    Serial2.begin(230400);

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

    /* Updates */
    comTeensy.update();
    lidar.update();

    /* Handle the lidar messages */
    if (lidar.waitingRX()) {
        lidar_frame = lidar.peekOldestFrame();

        angle_step = (float) (lidar_frame.end_angle - lidar_frame.start_angle) * 0.01f / (FRAME_NPOINT - 1.0f);
        for (point_id = 0; point_id < FRAME_NPOINT; point_id++) {
            distance = lidar_frame.data[point_id].distance;
            // if (distance > 100 && distance < 300) {
                angle = ((float) lidar_frame.start_angle + angle_step * (float) point_id) * DEG_TO_RAD;
                Serial.println("> xy :" + String(distance / 1000.0f * std::cos(angle),3) + ":" + String(distance / 1000.0f * std::sin(angle),3) + "|xy");
            // }
        }

        lidar.popOldestFrame();
    }

}
