#ifndef PoluloMotor_h
#define PoluloMotor_h

#include "Arduino.h"
#include "PoluloEncoder.h"

class PoluloMotor {
  public:
    PoluloMotor(int encoderPin1, int encoderPin2, int enablePin, int dirPin1, int dirPin2, float ratio);
    void setTargetVel(float targetVel);
    void releaseMotor() { engaged = false; }
    void engageMotor() { engaged = true; }
    float getTargetVel() { return target_vel; }
    long getTics(){ return encoder.getTics();}
    void pid();
    float getVel() {return encoder.getVel();}
    void setkP(float kp) { this->kp =  kp;}
    void setkI(float ki) { this->ki = ki;}
    void setkD(float kd) { this->kd = kd;}
    void setPID(byte kp_b, byte ki_b, byte kd_b) {
      this->kp =  ((unsigned int)kp_b) - 128;
      this->kd = ((unsigned int)kd_b) - 128;
      integral = 0;
      derivative = 0;
//      Serial.println(kp);
      }
    bool autoTune();
    void printTunedKs();
  private:
    PoluloEncoder encoder;
    
    float kp = 40;
    float ki = 5;
    float kd = 0.0;

    float MAX_DIFF_DROP_I = 1.0;

    int maxPWM = 255;
    int minPWM = 20;
    
    //Define Variables we'll be connecting to
    float target_vel;
    float integral;
    float derivative;
    float prev_err;
    bool engaged;

        
    int enablePin;
    int dirPin1;
    int dirPin2;
    // Model Vel when inputed the calibrating pwm - default value 
    float ffModelVel = 6.0f;

};

#endif
