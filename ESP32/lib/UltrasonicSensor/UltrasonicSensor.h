
#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_


class ultrasonicSensor{
    
    public:
    ultrasonicSensor();
    ultrasonicSensor(int pinTrig, int pinEcho);


    int distance(); 


    int dist;
    
    
    private : 
    int pinTrig, pinEcho;
    long duration;
};

#endif