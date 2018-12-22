#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000 
#include <util/delay.h>                // for _delay_ms()

#define PORT_LED PORTB
#define DDR_LED	 DDRB
#define PORT_LED_PIN PORTB0

#define DDR_QUAD DDRB
#define PORT_QUAD PORTB
#define PORT_QUAD_A PORTB1
#define PORT_QUAD_B PORTB2
#define PIN_QUAD PINB
#define PIN_QUAD_A PINB1
#define PIN_QUAD_B PINB2
#define INT_QUAD_A PCINT9
#define INT_QUAD_B PCINT10

static uint8_t pin_quad;

static int32_t counter;

int main()
{
	// Set led as output
	DDR_LED |= 1 << PORT_LED_PIN;

	// Enable interrupts
	sei();

	// Set quad pins as input
	DDR_QUAD &= ~(1 << PORT_QUAD_A);
	DDR_QUAD &= ~(1 << PORT_QUAD_B);

	// Set variables before interrupts
	pin_quad = PIN_QUAD;
	counter = 0;

	// Enable changes in pins of channel A and B
	PCMSK1 |= (1<<INT_QUAD_A);
	PCMSK1 |= (1<<INT_QUAD_B);		

	// Enable Interrupts for changes in pins 8:11
	GIMSK |= (1<<PCIE1);
	// Enable general interrupts
	sei();
	
	// Turn on the led if the counter is positve
	while (1) {
		if (counter > 0)
			PORT_LED |= 1 << PORT_LED_PIN;
		else
			PORT_LED &= ~(1 << PORT_LED_PIN);
		_delay_ms(50);
	}
}

// Interrupt routine for changes in pins 8:11
ISR(PCINT1_vect){
	// http://www.me.unm.edu/~starr/teaching/me470/quad.pdf
	// Read new state
	uint8_t new_pin_quad = PIN_QUAD;

	// b_old ^ a_new == 1 iff dir is pos
	uint8_t b_old = pin_quad >> PIN_QUAD_B;
	uint8_t a_new = new_pin_quad >> PIN_QUAD_A;
	uint8_t dir = (a_new ^ b_old) & 1;
	if (dir)
		counter++;
	else
		counter--;

	// Save new state as old
	pin_quad = new_pin_quad;
}
