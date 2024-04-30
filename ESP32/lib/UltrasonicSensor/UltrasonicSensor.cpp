#include "UltrasonicSensor.h"
#include "Arduino.h"

ultrasonicSensor::ultrasonicSensor(){};

ultrasonicSensor::ultrasonicSensor(int pinTrig, int pinEcho){
    this->pinTrig = pinTrig;
    this->pinEcho = pinEcho;
    pinMode(pinTrig, OUTPUT); // Sets the trigPin as an Output
    pinMode(pinEcho, INPUT); // Sets the echoPin as an Input

}


int ultrasonicSensor::distance(){


    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(pinEcho, HIGH, 100000);
    // Calculating the distance
    dist = duration * 0.034 / 2;
    return dist;
}