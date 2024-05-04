#include <Arduino.h>
#include <FastLED.h>
#include "LiquidCrystal_I2C.h"
#include "Communication.h"
#include "Message.h"
#include "IDS.h"
#include "ldlidar_driver_linux.h"

#define LED_PIN     18
#define NUM_LEDS    41
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

LiquidCrystal_I2C lcd(0x27, 16, 2);

Communication comTeensy = Communication(&Serial);
Message teen_msg;

uint64_t GetTimestamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

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

    ldlidar::LDLidarDriverLinuxInterface* lidar_drv = ldlidar::LDLidarDriverLinuxInterface::Create();
    lidar_drv->RegisterGetTimestampFunctional(std::bind(&GetTimestamp)); 
    lidar_drv->EnablePointCloudDataFilter(true);
    lidar_drv->Connect(ldlidar::LDType::LD_19);
    lidar_drv->WaitLidarComm(3500);
    lidar_drv->Start();

    ldlidar::Points2D laser_scan_points;
    int cnt = 100;
    while (ldlidar::LDLidarDriverLinuxInterface::Ok()) {
        if ((cnt--) <= 0) {
            lidar_drv->Stop();
        }

        switch (lidar_drv->GetLaserScanData(laser_scan_points, 1500)){
            case ldlidar::LidarStatus::NORMAL:
                //  output 2d point cloud data
    lcd.setCursor(0,1);
    lcd.print(String(laser_scan_points.size()));
                // for (auto point : laser_scan_points) {
                //     LOG_INFO_LITE("stamp(ns):%lu,angle:%f,distance(mm):%d,intensity:%d", 
                //         point.stamp, point.angle, point.distance, point.intensity);
                // }
                break;
            case ldlidar::LidarStatus::DATA_TIME_OUT:
                lidar_drv->Stop();
                exit(EXIT_FAILURE);
                break;
            case ldlidar::LidarStatus::DATA_WAIT:
                break;
            default:
                break;
        }

        delay(166);  // sleep 166ms , 6hz
    }


    lidar_drv->Stop();
    lidar_drv->Disconnect();
    ldlidar::LDLidarDriverLinuxInterface::Destory(lidar_drv);
}

void loop() {

    /* Update the Teensy communication */
    comTeensy.update();

    /* Handle the lidar messages */
    // lidar_msg = "";
    // chr_tmp = ' ';
    // while (Serial2.available() && chr_tmp != '\n') {
    //     chr_tmp = (char)Serial2.read();
    //     lidar_msg += chr_tmp;
    // }
    // if (lidar_msg.length() > 0) {
    //     uint8_t lidar_bytes [lidar_msg.length()];
    //     lidar_msg.getBytes(lidar_bytes, lidar_msg.length());
    //     if (lidar.Parse(lidar_bytes, lidar_msg.length())) {
    //         lidar.AssemblePacket();
    //     }
    // }

}
