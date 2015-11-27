#ifndef PoluloMotor_h
#define PoluloMotor_h

#include "Arduino.h"
#include "PoluloEncoder.h"

#include <PID_v1.h>

class PoluloMotor {
  public:
    PoluloMotor(int encoderPin1, int encoderPin2, int enablePin, int dirPin1, int dirPin2, float ratio);
    void setTargetVel(float targetVel);
    void pid();
    float getVel() {return encoder.getVel();}
    void setkP(float kp) { this->kp =  kp;}
    void setkI(float ki) { this->ki = ki;}
    void setkD(float kd) { this->kd = kd;}
  private:
    PoluloEncoder encoder;
    double kp = 40;
    double ki = 10;
    double kd = .5;

    //Define Variables we'll be connecting to
    double Input, Output, targetVel;;

    int enablePin;
    int dirPin1;
    int dirPin2;
    PID myPID;
};

#endif
