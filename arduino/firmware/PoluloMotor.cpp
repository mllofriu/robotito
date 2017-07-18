#include "PoluloMotor.h"


//Specify the links and initial tuning parameters

PoluloMotor::PoluloMotor(int encoderPin1, int encoderPin2, int enablePin, int dirPin1, int dirPin2, float ratio)
	: encoder(encoderPin1, encoderPin2, ratio)
{
	this->enablePin = enablePin;
	this->dirPin1 = dirPin1;
	this->dirPin2 = dirPin2;

  this->engaged = true;

	target_vel = 0;
  integral = 0;
  derivative = 0;
  prev_err = 0;
//
//  pidTuner.SetOutputStep(50);
//  pidTuner.SetControlType(1);
//  pidTuner.SetNoiseBand(1);

	pinMode(enablePin, OUTPUT);
	pinMode(dirPin1, OUTPUT);
	pinMode(dirPin2, OUTPUT);

	digitalWrite(enablePin, LOW);
	digitalWrite(dirPin1, LOW);
	digitalWrite(dirPin2, LOW);	
}

void PoluloMotor::printTunedKs(){
//  Serial.println("Constants");
//  Serial.println(pidTuner.GetKp());
//  Serial.println(pidTuner.GetKi());
//  Serial.println(pidTuner.GetKd());
}

bool PoluloMotor::autoTune()
{
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);  
  analogWrite(enablePin, 100);

  delay (300);
  
  long startTime = millis();
  // Discard one read
  encoder.getVel();
  float avgVel = encoder.getVel();
  while (millis() - startTime < 300){
    avgVel = .5 * encoder.getVel() + .5 * avgVel;
    delay(20);
  }
       
  ffModelVel = avgVel;    
  Serial.println(ffModelVel);

  analogWrite(enablePin, 0);
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  delay(300);
  return true;
}

void PoluloMotor::setTargetVel(float vel)
{
	target_vel = vel;
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
  if (!engaged){
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW); 
    digitalWrite(enablePin, LOW);
    return;
  }
  
  float curr_vel = encoder.getVel();
  float error = target_vel -  curr_vel;
  integral = integral + error;
  // Don't allow integral to grow so that overpowers the motor
  integral = abs(integral) > maxPWM / ki ? maxPWM / ki * sign(integral) : integral;
  derivative = (prev_err - error);
  prev_err = error;
  
  float ctrlSignal = kp * error + ki * integral + kd * derivative;
  // Direction setting
  if (abs(ctrlSignal) < minPWM){
//    digitalWrite(dirPin1, HIGH);
//    digitalWrite(dirPin2, HIGH); 
  } else if (ctrlSignal > 0){
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW); 
  } else if (ctrlSignal < 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH); 

  }
//  Serial.println("Control variables");
//  Serial.println(Input);
//  Serial.println(targetVel);
//  Serial.println(Output);
//  Serial.println(ff);

  ctrlSignal = abs(ctrlSignal) < minPWM ? 0 : ctrlSignal;
  
  analogWrite(enablePin, min(abs(ctrlSignal), maxPWM));
}
