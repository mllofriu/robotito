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

#include "pin_config.hpp"
  
void receive_cb(uint8_t num_bytes)
{
  uint8_t b = TinyWireS.read();
  if (b == 5U)
	  HB_P |= 1 << HB_N;
  else 
    HB_P &= ~(1 << HB_N);
}

int i = 0;
void request_cb()
{
  i++;
  TinyWireS.write(i++);
  TinyWireS.write(i);
}

void setup_motors()
{
	DDRA = (1 << M1_DIRA_N) | (1 << M1_DIRB_N) | (1 << M2_DIRA_N) | (1 << M2_DIRB_N);
	DDRB |= 1 << M1_EN_N;
	DDRB |= 1 << M2_EN_N;

  TCCR1A = (1 << COM1B1) | (0 << COM1B0) | (1 << PWM1B);
	TCCR1C |= (1 << COM1D1) | (0 << COM1D0) | (1 << PWM1D);
	TCCR1B = (1 << CS10);
  TCCR1D = (0 << WGM11) | (0 << WGM10);
}

int main()
{
  // Setup communications
  TinyWireS.begin(SLAVE_ADDR);
  TinyWireS.onReceive(receive_cb);
  TinyWireS.onRequest(request_cb);

  // Enable HeartBeat
	HB_DDR |= 1 << HB_N;
	HB_P &= ~(1<<HB_N);

  // Enable PWM
  setup_motors();

  sei();

	while(1)
  {
    TinyWireS_stop_check();
  }
}
