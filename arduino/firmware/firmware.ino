#include <XBee.h>
#include <SoftwareSerial.h>
#include "DSensorManager.h"
#include "MotorManager.h"


// Xbee objects
SoftwareSerial xbeeserial(0,1);
XBee xbee = XBee();

uint8_t option = 0;
uint8_t data = 0;


// Control variables
DSensorManager dsMgr;
MotorManager mMgr;


int CONTROL_PERIOD = 10000;

long XBEE_ADDR = 0x1111;


void setup() {
  xbeeserial.begin(57600);
  xbee.setSerial(xbeeserial);
}

void loop() {
  unsigned long wakeuptime = micros();
  
  dsMgr.process(xbee);
  mMgr.process(xbee);
  
  delayMicroseconds(CONTROL_PERIOD - (micros() - wakeuptime));
}
