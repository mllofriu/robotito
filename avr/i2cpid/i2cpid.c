/*
  Goal ... have the ATTiny84 join an I2C network as a slave 
  with an Arduino UNO as the I2C network master

  using the information from 
  http://thewanderingengineer.com/2014/02/17/attiny-i2c-slave/
  as the starting point.

*/

#include <avr/io.h>
#define F_CPU 8000000 
#include <util/delay.h>                // for _delay_ms()
#include <avr/interrupt.h>


#define SLAVE_ADDR 0xA   //this slave address (0x1, 0x2, 0x3)
#include "TinyWireS/TinyWireS.h"          //the ATTiny Wire library

#define HB_DDR DDRB
#define HB_P PORTB
#define HB_N PORTB6
  
void receive_cb(uint8_t num_bytes)
{
  uint8_t b = TinyWireS.read();
  if (b == 5U)
	  HB_P |= 1 << HB_N;
  else 
    HB_P &= ~(1 << HB_N);
}

int main()
{
  TinyWireS.begin(SLAVE_ADDR);
  TinyWireS.onReceive(receive_cb);

	HB_DDR |= 1 << HB_N;
	HB_P &= ~(1<<HB_N);

  sei();

	while(1)
  {
    TinyWireS_stop_check();
  }
}
