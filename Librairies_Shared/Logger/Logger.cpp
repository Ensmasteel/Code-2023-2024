#include "Logger.h"

//----------Logger Class----------//
           
Print* Logger::serialInfo;                 
Print* Logger::serialDebug;                
  
bool Logger::infoOpened;
bool Logger::debugOpened;

void Logger::setup(Print* infoSerial, Print* debugSerial, bool info, bool debug){
    serialInfo=infoSerial;
    serialDebug=debugSerial;
    infoOpened=info;
    debugOpened=debug;
    if (info){
        serialInfo->println("Le canal Info est ouvert");
    }
    if (debug){
        serialDebug->println("Le canal Debug est ouvert");
    }
}

void Logger::infoln(const String &message){
    if (infoOpened){
        serialInfo->println("Info : " + message);
    }
}

void Logger::debugln(const String &message){
    if (debugOpened){
        serialDebug->println("Debug : " + message);
    }
}

//----------End Logger Class----------//