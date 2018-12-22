/*
  Goal ... have the ATTiny84 join an I2C network as a slave 
  with an Arduino UNO as the I2C network master

  using the information from 
  http://thewanderingengineer.com/2014/02/17/attiny-i2c-slave/
  as the starting point.

*/

#include <avr/io.h>
#define F_CPU 1000000 
#include <util/delay.h>                // for _delay_ms()

#define I2C_SLAVE_ADDRESS 0xA   //this slave address (0x1, 0x2, 0x3)
#include "TinyWireS/utility/USI_TWI_Slave.h"          //the ATTiny Wire library
  
void read_cb(uint8_t num_bytes)
{
	// LED on
  PORTB = 0b00000001;            // PC0 = High = Vcc
}

int main()
{
	usiTwiSlaveInit(I2C_SLAVE_ADDRESS);
	DDRB = 0x01;
	PORTB = 0b00000000;
	while(1)
  {
		// put your main code here, to run repeatedly:
    if (!(USISR & (1 << USIPF))) continue;                    // Stop not detected
    uint8_t amount = usiTwiAmountDataInReceiveBuffer();
    if (amount == 0) continue;                                // no data in buffer
    read_cb(amount);
  }
}
