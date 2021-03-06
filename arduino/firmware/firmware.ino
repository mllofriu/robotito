#include <XBee.h>
#include <SoftwareSerial.h>
#include "DSensorManager.h"
#include "MotorManager.h"


// Xbee objects
SoftwareSerial xbeeserial(0,1);
XBee xbee = XBee();

// Control variables
DSensorManager dsMgr;
MotorManager mMgr;


long period;


void setup() {
  Serial.begin(9600);
  
  xbeeserial.begin(57600);
  xbee.setSerial(xbeeserial);

  // Get the loop period as the minimum of all managers
  period = min(dsMgr.getPeriod(), mMgr.getPeriod());

  // Blink
  for (int i = 0; i < 10; i++){
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  
}

void loop() {
  unsigned long wakeuptime = micros();
  
  dsMgr.process(xbee);
  mMgr.process(xbee);

  delayMicroseconds(period - (micros() - wakeuptime));

  Serial.println();
}
