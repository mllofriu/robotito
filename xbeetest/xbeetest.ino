
#include <SoftwareSerial.h>
SoftwareSerial serial(0,1);

void setup() {
  // put your setup code here, to run once:
  serial.begin(9600);
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  serial.println("Hello");
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  if (serial.available()) {
    Serial.write(serial.read());
  }
  
}
