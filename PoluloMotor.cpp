#include "PoluloMotor.h"


PoluloMotor::PoluloMotor(int encoderPin1, int encoderPin2, int enablePin, int dirPin1, int dirPin2, float ratio)
	: encoder(encoderPin1, encoderPin2, ratio)
{
	this->enablePin = enablePin;
	this->dirPin1 = dirPin1;
	this->dirPin2 = dirPin2;

	targetVel = 0;

	pinMode(enablePin, OUTPUT);
	pinMode(dirPin1, OUTPUT);
	pinMode(dirPin2, OUTPUT);

	digitalWrite(enablePin, LOW);
	digitalWrite(dirPin1, LOW);
	digitalWrite(dirPin2, LOW);	
}

void PoluloMotor::setIWindow(size_t winSize) {
    this->iWinSize = winSize;
    // errors = BoundedQueue<float>(iWinSize);
}

void PoluloMotor::setTargetVel(float vel)
{
	targetVel = vel;
	if (vel < 0){
		digitalWrite(dirPin1, LOW);
		digitalWrite(dirPin2, HIGH);	
	} else if (vel > 0) {
		digitalWrite(dirPin1, HIGH);
		digitalWrite(dirPin2, LOW);	
	} else {
		digitalWrite(dirPin1, HIGH);
		digitalWrite(dirPin2, HIGH);	
	}

	// errors = BoundedQueue<float>(iWinSize);

	pid();
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
	float currVel = encoder.getVel();
	float err = targetVel - currVel;

	// errors.enqueue(err);

  	// Serial.println(errors.average());
  	// Serial.println();
  	//  + errors.average() * i
	float gain = max(0, min(255, (p * err) * sign(targetVel)));
	// float gain = 30;
	// gain = 00;

	analogWrite(enablePin, gain);
}