#include "YDLidarG4.h"
//#include <Arduino.h>


YDLidarG4::YDLidarG4(){
    binedSerial = NULL;
    point.distance = 0.0;
    point.angle = 0.0;
    //point.quality = 0;

    //if a receive buffer already exists, we delete it
    if (receiveBuffer)
    {
        delete[] receiveBuffer;
        receiveBuffer = NULL;
    }

    receiveBuffer = new uint8_t[sizeof(singlePacketPoint)];

};


YDLidarG4::~YDLidarG4() {
  end();
  delete[] receiveBuffer;
}

// check whether the serial interface is opened
bool YDLidarG4::isOpen(void) {
  return binedSerial ? true : false;
}

bool YDLidarG4::begin(HardwareSerial &serialobj, uint32_t baudrate) {
  if (isOpen()) {
    end();
  }

  binedSerial = &serialobj;
  binedSerial->end();
  binedSerial->begin(baudrate);
  while (binedSerial->available()){
    printf("BYTE REMOVED FROM BUFFER\n");
    binedSerial->read();
  }
  return true;
}


void YDLidarG4::end(void){
    if (isOpen()){
        binedSerial->end();
        binedSerial = NULL;
    }
}


bool YDLidarG4::sendCommand(uint8_t cmd){
    cmdPacket pktHeader;
    cmdPacket *header = &pktHeader; //this is used to only pass the struct as a pointer and not copy eveything multiple times in memory 
    uint8_t checksum = 0;

    header->syncByte = LIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd & 0xff; //some hex trickery I will not try to understand
                                    //some 5 minutes of research seem to point out that this is used to mask the bits above and only keep the last 8 bits

    binedSerial->write((uint8_t *)header, 2);

    // Most of the code after that was for the payload and I don't think it was used anytime

    if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD)) {
        //printf("IN IFFF");
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= (cmd & 0xff);
    checksum ^= (0 & 0xFF);


    uint8_t sizebyte = 0;
    binedSerial->write(&sizebyte, 1);
    //binedSerial->write((const uint8_t *)NULL, sizebyte);
    binedSerial->write(&checksum, 1);
  }

    return true;
}

bool YDLidarG4::stop(void){
    
    if (!isOpen()){
        return false;
    }

    bool ans = true;
    ans = sendCommand(LIDAR_CMD_FORCE_STOP);

    return ans;
}


bool YDLidarG4::startScan(bool force, uint32_t timeout){
    
    //printf("LIDAR_STARTSCAN()");

    bool ans;
    
    if (!isOpen()){
        //printf("Failed -1");
        return false;
    }
    stop();

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) == false){
            //printf("Failed 0");
        return ans;
    }
    lidarAnsHeader responseHeader;
    if ((ans = waitResponseHeader(&responseHeader, LIDAR_ANS_TYPE_MEASUREMENT, timeout)) == false){
            //printf("Failed 1");
        return ans;
    }

    if (responseHeader.size < 4){

        //printf("Failed 2");
        return false;
    }

    return true; 
}




/*
bool YDLidarG4::waitScanDot(uint32_t timeout){



    
    static uint16_t package_Sample_Index = 0;
    static uint8_t sampleNumPackage = 0;


    static uint8_t package_CT = 0;
      
    static uint16_t FirstSampleAngle = 0;
    static uint16_t LastSampleAngle = 0;
    static uint16_t CheckSum = 0;

    
    static uint16_t checkSumCal = 0;
    static uint16_t SampleNumlAndCTCal = 0;
    static uint16_t LastSampleAngleCal = 0;
  
    static bool checksumOK = true;

    static float IntervalSampleAngle = 0;
    static float IntervalSampleAngle_LastPackage = 0;

    uint32_t startTime = millis();
    uint32_t waitTime = 0;
    int recvPos = 0;

    int packageRecvPos = 0;
    static uint16_t value8to16;

    int32_t AngleCorrectForDistance = 0;

    node_info node;
    node_package* package = (node_package*)receiveBuffer;


    // if there si no connection, we just retrurn false, as an error 
    if (!isOpen())
        return false;

//printf("AAAA\n");

if (package_Sample_Index == 0)
  {
    //printf("BBB\n");
    recvPos = 0;

    while((waitTime = millis() - startTime) <= timeout)
    {

        int currentByte = binedSerial->read();                                              // In the librairy, they use a uint16_t, but only a uint8_t should be needed
        //printf("CCC\n");

    if (currentByte<0){
        continue;
    }

    switch(recvPos)
    {
        case 0 :
            //printf("DDD\n");
            if (currentByte != PH1)
                continue;
            break;
        
        case 1 :
            //printf("EEE\n");
            checkSumCal = PH;
            if (currentByte != PH2){
                recvPos = 0;
                continue;
            }
            break;
        

        case 2:
            //printf("FFF\n");
            SampleNumlAndCTCal = currentByte;
            if (((currentByte & 0x01) != CT_Normal) && ((currentByte & 0x01 != CT_RingStart))){
                recvPos = 0;
                continue;
            }
            package_CT = currentByte;   //CT = 0x00 means point cloud data
                                        //CT = 0x01 means the begining data packet
            break;
    
        case 3:
            //printf("GGG\n");
            SampleNumlAndCTCal += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT); 
            sampleNumPackage = currentByte;
        break;

        case 4:
            //printf("HHH\n");
            if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT){
                FirstSampleAngle = currentByte;
            }else{
                recvPos = 0;
                continue;
            }

        case 5:
            //printf("III\n");
            FirstSampleAngle += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
            checkSumCal ^= FirstSampleAngle;
            FirstSampleAngle = FirstSampleAngle >> 1;
            break;

        case 6:
            //printf("JJJ\n");
            if(currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT){
                LastSampleAngle = currentByte;
            }else{
                recvPos = 0;
                continue;
            }
            break;

        case 7:
            LastSampleAngle += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
            LastSampleAngleCal = LastSampleAngle;
            LastSampleAngle = LastSampleAngle >> 1;

            if (sampleNumPackage == 1){
                IntervalSampleAngle = 0;
            }else {
                if (LastSampleAngle < FirstSampleAngle)
                {
                    if ((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)){
                        IntervalSampleAngle = ((float) (23040 + LastSampleAngle - FirstSampleAngle)) / (sampleNumPackage - 1);
                        IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                    }else{
                        IntervalSampleAngle = IntervalSampleAngle_LastPackage;
                    }
                }else{
                    IntervalSampleAngle = ((float)(LastSampleAngle - FirstSampleAngle)) / (sampleNumPackage - 1);
                    IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                }

            }
            break;

        //byte where the checksum is stored
        case 8:
            CheckSum = currentByte;
            break;
        case 9:
            CheckSum += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);

    }


    //printf("KKK\n");

    receiveBuffer[recvPos++] = currentByte;

    if (recvPos == PackagePaidBytes){
        packageRecvPos = recvPos;
        break;
    }

    //printf("LLL\n");

    }

//printf("MMM\n");
    if (recvPos == PackagePaidBytes){
        startTime = millis();
        recvPos = 0;
        int packageSampleSum = sampleNumPackage*2;

        //printf("NNN\n");
        while((waitTime = millis() - startTime) <= timeout){
            int currentByte = binedSerial->read();
            if (currentByte < 0 ){
                continue;
            }
            
            if (recvPos%2 == 0){
                value8to16 = currentByte;
            }
            else{
                value8to16 += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
                checkSumCal ^= value8to16;
            }

            receiveBuffer[packageRecvPos + recvPos] = currentByte;
            recvPos ++;

            if (packageSampleSum == recvPos){
                packageRecvPos += recvPos;
                break;
            }
            //printf("OOO\n");
        }
    if (packageSampleSum != recvPos){
        return false;
    }
    } else{
        return false;
    }

    //printf("PPP\n");
    checkSumCal ^= SampleNumlAndCTCal;
    checkSumCal ^= LastSampleAngleCal;

    if (checkSumCal != CheckSum){
        printf("CheckSum error cal: %04X cur: %04X\n", checkSumCal, CheckSum);
        checksumOK = false;
    }else{
        checksumOK = true;
    }
}
    //removed sync flags things
//printf("QQQ\n");
      if ((package_CT&0x01) == CT_Normal) {
        node.sync_flag = Node_NotSync;
    } else {
        node.sync_flag = Node_Sync;
    }



    //printf("we are going to enter in if(checksum)\n");
    if (checksumOK){
        //printf("We are in the if");

        node.distance = package->points[package_Sample_Index].dist;

        if (node.distance != 0){
            AngleCorrectForDistance = (int32_t)((atan(((21.8 * (155.3 - (node.distance)))/155.3) / (node.distance))) * 3666.93);
        }else{
            AngleCorrectForDistance = 0;
        }

        float sampleAngle = IntervalSampleAngle * package_Sample_Index;

        if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0){
            node.angle = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } else{
            if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040){
                node.angle = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } else{
                node.angle = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }
        }
    }else{
        node.sync_flag = Node_NotSync;
        node.angle = LIDAR_RESP_MEASUREMENT_CHECKBIT;
        node.distance = 0;
        package_Sample_Index = 0;
        return false;
    }

    point.distance = node.distance;
    point.angle = (node.angle >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
    point.startBit = (node.sync_flag);

    package_Sample_Index ++;

    
    
    //if the current index is greater that the message length, we reset it back to 0
    if (package_Sample_Index >= sampleNumPackage){
        package_Sample_Index = 0;
    }

    return true;
}

*/




bool YDLidarG4::waitResponseHeader(lidarAnsHeader *header, uint8_t cmd, uint32_t timeout){
    uint8_t recvPos = 0; //Changed from int
    uint32_t startTs = millis();
    uint8_t *headerBuffer = (uint8_t *)(header);


    while ((millis() - startTs) <= timeout){
        uint8_t currentbyte = binedSerial->read();

        if (currentbyte < 0)
            continue;

        switch (recvPos)
        {
        case 0:
            if (currentbyte != LIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != LIDAR_ANS_SYNC_BYTE2){
                recvPos = 0 ;
                continue;
            }
            break;
        case 6:
            if (cmd && currentbyte != cmd){
                recvPos = 0;
                continue;
            }
            
            break;
        }

        headerBuffer[recvPos++] = currentbyte;

        if (recvPos == sizeof(lidarAnsHeader)){
            return true;
        }
    }
    
    return true;
}


bool YDLidarG4::waitScanDot(uint32_t timeout) 
{
  if (!isOpen())
    return false;

  int recvPos = 0;
  uint32_t startTs = millis();
  uint32_t waitTime = 0;
  node_info node;
  node_package* package = (node_package*)receiveBuffer;
  
  static uint16_t package_Sample_Index = 0;
  static float IntervalSampleAngle = 0;
  static float IntervalSampleAngle_LastPackage = 0;
  static uint16_t FirstSampleAngle = 0;
  static uint16_t LastSampleAngle = 0;
  static uint16_t CheckSum = 0;

  static uint16_t CheckSumCal = 0;
  static uint16_t SampleNumlAndCTCal = 0;
  static uint16_t LastSampleAngleCal = 0;
  static bool CheckSumResult = true;
  static uint16_t Valu8Tou16 = 0;

  static uint8_t package_Sample_Num = 0;
  int32_t AngleCorrectForDistance = 0;
  static uint8_t package_CT = 0;
  int package_recvPos = 0;

  if (package_Sample_Index == 0)
  {
    recvPos = 0;

    while ((waitTime = millis() - startTs) <= timeout) 
    {
      int currentByte = binedSerial->read();

      if (currentByte < 0)
        continue;

      //printf("%02X ", uint8_t(currentByte));

      switch (recvPos)
      {
      case 0:
        if (currentByte != PH1) {
          continue;
        }
        break;

      case 1:
        CheckSumCal = PH;
        if (currentByte != PH2) {
          recvPos = 0;
          continue;
        }
        break;

      case 2:
        SampleNumlAndCTCal = currentByte;
        if (((currentByte&0x01) != CT_Normal) && ((currentByte & 0x01) != CT_RingStart)) {
          recvPos = 0;
          continue;
        }
        package_CT = currentByte;
        break;

      case 3:
        SampleNumlAndCTCal += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        package_Sample_Num = currentByte;
        break;

      case 4:
        if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          FirstSampleAngle = currentByte;
        } else {
          recvPos = 0;
          continue;
        }
        break;  

      case 5:
        FirstSampleAngle += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        CheckSumCal ^= FirstSampleAngle;
        FirstSampleAngle = FirstSampleAngle >> 1;
        break;

      case 6:
        if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          LastSampleAngle = currentByte;
        } else {
          recvPos = 0;
          continue;
        }
        break;

      case 7:
        LastSampleAngle += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        LastSampleAngleCal = LastSampleAngle;
        LastSampleAngle = LastSampleAngle >> 1;

        if (package_Sample_Num == 1) {
          IntervalSampleAngle = 0;
        } else {
          if (LastSampleAngle < FirstSampleAngle) {
            if ((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)) {
              IntervalSampleAngle = ((float)(23040 + LastSampleAngle - FirstSampleAngle)) /
                                    (package_Sample_Num - 1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            } else {
              IntervalSampleAngle = IntervalSampleAngle_LastPackage;
            }
          } else {
            IntervalSampleAngle = ((float)(LastSampleAngle - FirstSampleAngle)) / (package_Sample_Num - 1);
            IntervalSampleAngle_LastPackage = IntervalSampleAngle;
          }
        }
        break;

      case 8:
        CheckSum = currentByte;
        break;
      case 9:
        CheckSum += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        break;
      }

      receiveBuffer[recvPos++] = currentByte;

      if (recvPos == PackagePaidBytes) {
        package_recvPos = recvPos;
        break;
      }
    }



    if (PackagePaidBytes == recvPos) 
    {
      startTs = millis();
      recvPos = 0;
      int package_sample_sum = intensity ? package_Sample_Num * 3 : package_Sample_Num * 2;

      while ((waitTime = millis() - startTs) <= timeout) 
      {
        
        int currentByte = binedSerial->read();
        if (currentByte < 0)
          continue;

        //printf("#%02X #", uint8_t(currentByte));

        //Compute checksums for point cloud parts
        if (intensity)
        {
          if (recvPos % 3 == 0)
            CheckSumCal ^= uint8_t(currentByte);
          else if (recvPos % 3 == 1)
            Valu8Tou16 = currentByte;
          else
          {
            Valu8Tou16 += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
            CheckSumCal ^= Valu8Tou16;
          }
        }
        else
        {
          if (recvPos % 2 == 0)
            Valu8Tou16 = currentByte;
          else {
            Valu8Tou16 += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
            CheckSumCal ^= Valu8Tou16;
          }
        }

        receiveBuffer[package_recvPos + recvPos] = currentByte;
        recvPos ++;

        if (package_sample_sum == recvPos) {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_sample_sum != recvPos) {
        //printf("line414 RESULT FAIL package_sample_run \n");
        return false;
      }
    } else {
      printf("line418 RESULT FAIL else ? \n");
      return false;
    }

    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    if (CheckSumCal != CheckSum) {
      //printf("CheckSum error cal: %04X cur: %04X\n", CheckSumCal, CheckSum);
      CheckSumResult = false;
    } else {
      CheckSumResult = true;
    }
  }

  if ((package_CT&0x01) == CT_Normal) {
    node.sync_flag = Node_NotSync;
  } else {
    node.sync_flag = Node_Sync;
  }

  if (CheckSumResult)
  {
    //如果带信号强度信息
    
    if (true)
    {
      //node.quality = 0;
      node.distance = package->points[package_Sample_Index].dist;
    }

    if (node.distance != 0) {
      AngleCorrectForDistance = (int32_t)((atan(((21.8 * (155.3 - (node.distance))) /
         155.3) / (node.distance))) * 3666.93);
    } else {
      AngleCorrectForDistance = 0;
    }

    float sampleAngle = IntervalSampleAngle * package_Sample_Index;
    if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0) {
      node.angle = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance +
          23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
        node.angle = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance -
            23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        node.angle = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance)) <<
            LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    node.sync_flag = Node_NotSync;
    //node.quality = Node_Default_Quality;
    node.angle = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    node.distance = 0;
    package_Sample_Index = 0;
    return false;
  }

  point.distance = node.distance;
  point.angle = (node.angle >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
  //point.quality = (node.quality);
  point.startBit = (node.sync_flag);

  package_Sample_Index ++;
  if (package_Sample_Index >= package_Sample_Num) {
    package_Sample_Index = 0;
  }

  return true;
}