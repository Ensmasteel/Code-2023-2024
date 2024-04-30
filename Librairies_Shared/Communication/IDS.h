#ifndef IDS_H_
#define IDS_H_

#include <stdint.h>
#include "Arduino.h"
enum EquipmentID : uint8_t
{
    EmptyE,
    Teensy,
    Arduino,
    ESP_32,
    Raspberry
};

enum ActionID : uint8_t
{
    Pause,
    PaletMarron, //Recupere 3 palets marrons et les stocke dans la position marron
    PaletJaune, //Recupere 3 palets jaunes et les stocke dans la position jaune
    PaletRose, //Recupere 3 palets roses et les stocke dans la position rose
    Depot //Construit un (et un seul) gateau
};



enum DataID : uint8_t
{
    EmptyD,
    Coordonnees,    //Endroit
    Todo,           //Quelle action ?
    MessLidar,      //Message du Lidar
    MessActuator ,  //Message 
    EndAction
};

#endif
