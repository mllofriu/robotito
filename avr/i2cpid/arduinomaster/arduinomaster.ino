
#include <Wire.h>

#define SDA A4
#define SCL A5

void setup() {
  delay(100);

  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
//  pinMode(SDA, OUTPUT);
//  pinMode(SCL, INPUT_PULLUP);
//
//  delay(500);
//  digitalWrite(SDA, LOW);
}

uint8_t b = 5;
void loop() {
//  Serial.println("Sending Bytes");

//  Serial.println(Wire.write(0));              // sends one byte
//  Serial.println(Wire.write(1));  
//  Serial.println(Wire.endTransmission());    // stop transmitting
//  delay(500);
//  Wire.beginTransmission(10); // transmit to device #8
//  Wire.write(b);
 
//  Wire.endTransmission();
  Wire.requestFrom(10, 2);
  Serial.println(Wire.read());
  while (Wire.available() == 0);
  Serial.println(Wire.read());
//  
//  if (b == 5)
//    b = 2;
//  else
//    b = 5;

  delay(10);
}
