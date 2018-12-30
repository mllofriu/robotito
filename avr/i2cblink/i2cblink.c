#include "TinyWire/twi.h"
#include <avr/io.h>
#define F_CPU 8000000 
#include <util/delay.h>                // for _delay_ms()
#include <avr/interrupt.h>

#define HB_DDR DDRB
#define HB_P PORTB
#define HB_N PB6

static unsigned int own_address = 10;
volatile unsigned int delay = 5;

void onI2CReceive(int howMany){
	if(Twi_available()>0){	
		delay = Twi_receive();
		
	}
	HB_P |= 1 << HB_N;
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

	HB_DDR |= 1 << HB_N;

	sei();

	while(1)
  {
			//HB_P = 1 << HB_N;
			sleep_ms(delay);
			
			//HB_P &= ~(1 << HB_N);
			sleep_ms(delay);			
  }
}
