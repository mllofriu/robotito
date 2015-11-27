#include <PoluloMotor.h>
#include <Encoder.h>

#define MOVE_PERIOD 800
#define SLEEP_PERIOD 500
 
PoluloMotor * m1;
PoluloMotor * m2;
PoluloMotor * m3;

float motionVel = 8;    
float pVal = 50;
float iVal = 10;

char state;     
char prevState;                     
long lastUpdate;
int counter = 0;
void * pointer;
void setup() {
  Serial.begin(9600);


  
  m1 = new PoluloMotor(9, 8, 10, 11, 12,30.0f);
  m2 = new PoluloMotor(3, 2, 6, 5, 4, 51.45f);
  m3 = new PoluloMotor(16, 17, 20, 19, 18, 51.45f);

  m1->setP(pVal);
  m2->setP(pVal);
  m3->setP(pVal);

  m1->setI(iVal);
  m2->setI(iVal);
  m3->setI(iVal);
  
  state = 's';
  prevState = 'c';
  lastUpdate = millis();
}

void setVels(float v1, float v2, float v3){
  m1->setTargetVel(v1);
  m2->setTargetVel(v2);
  m3->setTargetVel(v3);
}

void loop() {
  m1->pid();

  m2->pid();
  m3->pid();
  Serial.println("PID 4 done");

  switch (state){
      case 's':
        if (millis() - lastUpdate > SLEEP_PERIOD){
          switch (prevState){
            case 'a':
              setVels(-motionVel, motionVel, 0);
              state = 'b';
              break;
            case 'b':
              setVels(motionVel, 0, -motionVel);
              state = 'c';
              break;
            case 'c':
              setVels(0, -motionVel, motionVel);
              state = 'a';
              break;
          }
          lastUpdate = millis();
        }
        break;
      case 'a':
      case 'b':
      case 'c':
        if (millis() - lastUpdate > MOVE_PERIOD){
          setVels(0, 0, 0);
          prevState = state;
          state = 's';
          lastUpdate = millis();
        }
        break;
  }
  

  pointer = malloc(1);
  counter++;
  if (pointer == NULL) Serial.println("got null");
  else Serial.println(String(counter) + " allocations made");
  delay(20);
}
