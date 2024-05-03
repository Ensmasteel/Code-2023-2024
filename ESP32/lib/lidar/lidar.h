// https://github.com/richardw347/ld19_lidar

#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <array>
#include <iostream>
#include <vector>
#include <functional>

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)
#define RADIAN_TO_ANGLE(angle) ((angle)*180000 / 3141.59)

enum {
    PKG_HEADER = 0x54,
    PKG_VER_LEN = 0x2C,
    POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t confidence;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructDef point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LiDARFrameTypeDef;

struct PointData {
    float angle;
    uint16_t distance;
    uint8_t confidence;
    double x;
    double y;
    PointData(float angle, uint16_t distance, uint8_t confidence, double x = 0, double y = 0) {
        this->angle = angle;
        this->distance = distance;
        this->confidence = confidence;
        this->x = x;
        this->y = y;
    }
    PointData() {}
    friend std::ostream &operator<<(std::ostream &os, const PointData &data) {
        os << data.angle << " " << data.distance << " " << (int)data.confidence << " " << data.x << " " << data.y;
        return os;
    }
};

class Lidar {
   public:
        Lidar();
        double GetSpeed(void) { return mSpeed; }            /*Lidar spin speed (Hz)*/
        uint16_t GetTimestamp(void) { return mTimestamp; } /*time stamp of the packet */
        bool IsFrameReady(void) { return mFrameReady; }    /*Lidar data frame is ready*/
        void ResetFrameReady(void) { mFrameReady = false; }
        long GetErrorTimes(void) { return mErrorTimes; } /*the number of errors in parser process of lidar data frame*/
        bool Parse(const uint8_t *data, long len);       /*parse single packet*/
        bool AssemblePacket();                           /*combine stantard data into data frames and calibrate*/
        void SetPopulateCallback(std::function<void(const std::vector<PointData> &laser_data)> callback) { mPopulateCallback = callback; }

   private:
        uint16_t mTimestamp;
        double mSpeed;
        std::vector<uint8_t> mDataTmp;
        long mErrorTimes;
        std::array<PointData, POINT_PER_PACK> mOnePkg;
        std::vector<PointData> mFrameTmp;
        bool mFrameReady;
        std::function<void(const std::vector<PointData> &laser_data)> mPopulateCallback;
};

class Tofbf {
    public:
        Tofbf() = delete;
        Tofbf(const Tofbf &) = delete;
        Tofbf & operator = (const Tofbf &) = delete;
        Tofbf(int speed);
        std::vector < PointData > NearFilter(const std::vector < PointData > &tmp) const;
        ~Tofbf();

    private:
        const int CONFIDENCE_LOW = 15;     /* Low confidence threshold */
        const int CONFIDENCE_SINGLE = 220; /* Discrete points require higher confidence */
        const int SCAN_FRE = 4500;         /* Default scan frequency, to change, read according to radar protocol */
        double offset_x, offset_y;
        double curr_speed;
};

#endif /* LIDAR_H */
