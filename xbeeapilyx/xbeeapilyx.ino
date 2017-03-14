#include <XBee.h>
#include <SoftwareSerial.h>
SoftwareSerial xbeeserial(0,1);
XBee xbee = XBee();

Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
uint8_t option = 0;
uint8_t data = 0;

void setup() {
  Serial.begin(9600);
  
  xbeeserial.begin(9600);
  xbee.setSerial(xbeeserial);
  
  // Create an array for holding the data you want to send.
  uint8_t payload[] = { 'H', 'i' };
  
  // Create a TX Request
  Tx16Request tx = Tx16Request(0x1111, payload, sizeof(payload));
  
  // Send your request
  xbee.send(tx);
}

void loop() {
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) {
    // got something
    
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      // got a rx packet
      
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
              xbee.getResponse().getRx16Response(rx16);
        option = rx16.getOption();
        data = rx16.getData(0);
      } else {
              xbee.getResponse().getRx64Response(rx64);
        option = rx64.getOption();
        data = rx64.getData(0);
      }
      
      Serial.write(data);
    }
  } 
    
}
