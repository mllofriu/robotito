#ifndef PoluloMotor_h
#define PoluloMotor_h

#include "Arduino.h"
#include "PoluloEncoder.h"

class PoluloMotor {
  public:
    PoluloMotor(int encoderPin1, int encoderPin2, int enablePin, int dirPin1, int dirPin2, float ratio);
    void setTargetVel(float targetVel);
    void pid();
    float getVel() {return encoder.getVel();}
    void setP(float p) { this->p =  p;}
    void setI(float i) { this->i = i;}
    void setIWindow(size_t winSize);
  private:
    PoluloEncoder encoder;
    float targetVel;
    float p = 100;
    float i = 10;

    size_t iWinSize = 10;

    int enablePin;
    int dirPin1;
    int dirPin2;
};

#endif