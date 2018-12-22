#include "TinyWire/twi.h"
#include <avr/io.h>
#define F_CPU 8000000 
#include <util/delay.h>                // for _delay_ms()

static unsigned int own_address = 10;
volatile unsigned int delay = 5;

void onI2CReceive(int howMany){
	if(Twi_available()>0){	
		delay = Twi_receive();
	}
}

void sleep_ms(unsigned int num_chunks){
	int i;
	for (i = 0; i < num_chunks; i++)
		_delay_ms(100);
}

int main()
{
	//Twi_attachSlaveTxEvent(onI2CReceive);
	Twi_attachSlaveRxEvent(onI2CReceive);
	Twi_slave_init(own_address);

	DDRB = 0x01;

	SREG = 1 << 7;

	while(1)
  {
			PORTB = 0b00000001;
			sleep_ms(delay);
			
			PORTB = 0b00000000;
			sleep_ms(delay);			
  }
}
