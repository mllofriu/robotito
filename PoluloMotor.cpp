#include "PoluloMotor.h"


//Specify the links and initial tuning parameters
PID * myPID;

PoluloMotor::PoluloMotor(int encoderPin1, int encoderPin2, int enablePin, int dirPin1, int dirPin2, float ratio)
	: encoder(encoderPin1, encoderPin2, ratio), myPID(&Input, &Output, &targetVel,kp,ki,kd, DIRECT)
{
	this->enablePin = enablePin;
	this->dirPin1 = dirPin1;
	this->dirPin2 = dirPin2;

	targetVel = 0;
  Input = encoder.getVel();
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(20); 
  myPID.SetMode(AUTOMATIC);

	pinMode(enablePin, OUTPUT);
	pinMode(dirPin1, OUTPUT);
	pinMode(dirPin2, OUTPUT);

	digitalWrite(enablePin, LOW);
	digitalWrite(dirPin1, LOW);
	digitalWrite(dirPin2, LOW);	
}

void PoluloMotor::setTargetVel(float vel)
{
	targetVel = vel;
}

int sign(float num){
	if (num > 0)
		return 1;
	else if (num < 0)
		return -1;
	else 
		return 0;
}

void PoluloMotor::pid()
{
  Input = encoder.getVel();
  myPID.Compute();
  if (Output < 0){
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);  
  } else if (Output > 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW); 
  } 
//  Serial.println(Input);
//  Serial.println(targetVel);
//  Serial.println(Output);
  analogWrite(enablePin, abs(Output));
}
