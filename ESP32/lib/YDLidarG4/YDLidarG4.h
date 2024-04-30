/*Librairy for YDLIDAR G4
*/

#pragma once

#ifndef YDLIDARG4_H_
#define YDLIDARG4_H_

#include "Arduino.h"


#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00

//#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x91

#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x8000

#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81


#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT    8

//response starting bytes are 0x55AA, so PH1 is received first then PH2
#define PH                                  0x55AA
#define PH1                                 0xAA
#define PH2                                 0x55

#define PackagePaidBytes 10
#define PackageSampleMaxLength 0x40

#define Node_Sync 1
#define Node_NotSync 2


//for debug
#define intensity 0

typedef enum {
  CT_Normal = 0,
  CT_RingStart  = 1,
  CT_Tail,
} CT;

struct node_info {
  uint8_t sync_flag;
  //Removed quality
  uint16_t angle;
  uint16_t distance;
} __attribute__((packed)) ;




struct scanPoint{
    float angle;
    float distance;
    bool startBit;
};


struct singlePacketHeader {
    uint16_t  package_Head;
    uint8_t   package_CT;
    uint8_t   nowPackageNum;
    uint16_t  packageFirstSampleAngle;
    uint16_t  packageLastSampleAngle;
    uint16_t  checkSum;
} __attribute__((packed));

struct singlePoint {
    uint16_t is : 2; //`is` is a bitField that has 2 bits allocated to it
    uint16_t dist : 14; // dist has however 14 bit allocated to it  (don't know, don't ask me why)
} __attribute__((packed));


struct node_package {
  singlePacketHeader head;
  singlePoint points[PackageSampleMaxLength];
} __attribute__((packed));


struct singlePacketPoint{
    singlePacketHeader head;

}__attribute__((packed));

struct cmdPacket {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

struct lidarAnsHeader {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} __attribute__((packed));




class YDLidarG4{

    public:
    enum{
    SERIAL_BAUDRATE = 230400,
    DEFAULT_TIMEOUT = 500
    };

    /**
    * Constructor of YDLIDARG
    */

    YDLidarG4();

    /**
     * Destructor of YDLIDARG4
    */
    ~YDLidarG4();

    
    /**
     * Open the given serial interface and try to start connection with YDLIDARG4
     * @returns Bollean if connection successfully established
    */
    bool begin(HardwareSerial &serialobj, uint32_t baudrate = SERIAL_BAUDRATE);

    /**
     * Close the currently open Serial connection
    */
    void end(void);

    /**
     * check whether the serial interface is opened or not
    */
    bool isOpen(void);

    /**
     * stop a scan
    */
    bool stop(void);



    bool sendCommand(uint8_t cmd);


    /**
     * @param force is a bollean describing whether or not a force command needs to be used for the LIDAR
     * @param timeout describe how longg we need to wait for an incomiing point, default is 500ms
     * @returns a boolean, `true` if scan successfully started, `false` if not 
    */
    bool startScan (bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);



    bool waitScanDot(uint32_t timeout = DEFAULT_TIMEOUT);


    const scanPoint &getCurrentScanPoint(void){
        return point;
    }


    protected:

        scanPoint point;
        bool waitResponseHeader(lidarAnsHeader *header, uint8_t cmd = 0, uint32_t timeout = DEFAULT_TIMEOUT);

        HardwareSerial *binedSerial = NULL;

        uint8_t *receiveBuffer = NULL;
        //uint8_t *packageBuffer = NULL;
};

#endif