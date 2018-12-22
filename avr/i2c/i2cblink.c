#define F_CPU 8000000 // or whatever may be your frequency
 
#include <avr/io.h>
#include <util/delay.h>                // for _delay_ms()

#include "usi_i2c_slave.h"

//Define a reference to the I2C slave register bank pointer array
extern char* USI_Slave_register_buffer[];

int main()
{
	DDRB = 0x01;
	// Activate interruptions
	SREG = 1 << 7;

	unsigned int delay = 0;

	USI_Slave_register_buffer[0] = (unsigned char*)&delay;

	//Initialize I2C slave with slave device address 0x40
	USI_I2C_Init(10);


	while(1)
  {
      // LED on
			if (delay != 0){
					PORTB = 0b00000001;  
			} else {
					PORTB = 0b00000000;  
			}
	    // PC0 = High = Vcc
			_delay_ms(10); 
  }
}
