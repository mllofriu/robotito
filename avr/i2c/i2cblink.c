#define F_CPU 8000000 // or whatever may be your frequency
 
#include <avr/io.h>
#include <util/delay.h>                // for _delay_ms()

#include "usi_i2c_slave.h"

#define HB_DDR DDRB
#define HB_P PORTB
#define HB_N PB6

//Define a reference to the I2C slave register bank pointer array
extern char* USI_Slave_register_buffer[];

int main()
{
	HB_DDR |= 1 << PB6;
	// Activate interruptions
	SREG = 1 << 7;

	unsigned int delay = 0;

	USI_Slave_register_buffer[0] = (char*)&delay;

	//Initialize I2C slave with slave device address 0x40
	USI_I2C_Init(10);


	while(1)
  {
      // LED on
			if (delay != 0){
					HB_P |= 1 << HB_N;  
			} else {
					HB_P &= ~(1 << HB_N);  
			}
	    // PC0 = High = Vcc
			_delay_ms(10); 
  }
}
