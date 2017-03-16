#include "PoluloMotor.h"
#include <Encoder.h>

#define MOVE_PERIOD 3000
 
PoluloMotor * m1;
PoluloMotor * m2;
PoluloMotor * m3;

float spinvel = 10;    
float kp = 10;
float ki = 50;
float kd = .5;


char state;     
long lastUpdate;
int counter = 0;
void * pointer;

bool autoTune1Completed = 1;


int m1en = 3;
int m2en = 4;
int m3en = 5;
int m4en = 6;

int m1i1 = 7;
int m1i2 = 8;
int m2i1 = 9;
int m2i2 = 10;
int m3i1 = 11;
int m3i2 = 12;
int m4i1 = 13;
int m4i2 = 14;

int m1enc1 = 15;
int m1enc2 = 16;
int m2enc1 = 17;
int m2enc2 = 18;
int m3enc1 = 19;
int m3enc2 = 20;
int m4enc1 = 21;
int m4enc2 = 22;


void setup() {
  Serial.begin(9600);


  
  m1 = new PoluloMotor(m1enc1, m1enc2, m1en, m1i1, m1i2, 51.45f);
  m2 = new PoluloMotor(m3enc1, m3enc2, m3en, m3i1, m3i2, 51.45f);
  m3 = new PoluloMotor(m4enc1, m4enc2, m4en, m4i1, m4i2, 51.45f);
  
  state = 's';
  lastUpdate = millis();

//   m1->setTargetVel(4);
  delay(1000);

  m1->autoTune();
  m2->autoTune();
  m3->autoTune();
}

void setVels(float v1, float v2, float v3){
  m1->setTargetVel(v1);
  m2->setTargetVel(v2);
  m3->setTargetVel(v3);
  
}

void loop() {
//  if (!autoTune1Completed)
//    autoTune1Completed = m1->autoTune();
//    
//  else {
////    m1->setTargetVel(4);
////    m1->printTunedKs();
//  }
  m1->pid();
  m2->pid();
  m3->pid();

  if (millis() - lastUpdate > MOVE_PERIOD){
    switch (state){
      case 's':
        state = 'h';
        break;
      case 'h':
        state = 's';
        break;
    }
    lastUpdate = millis();
  }

  switch(state) {
      case 's':
        setVels(spinvel, spinvel, spinvel);
        break;
      case 'h':
        setVels(-spinvel, -spinvel, -spinvel);
        break;
  }

  
    

  delay(20);
}
