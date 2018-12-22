#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000 
#include <util/delay.h>                // for _delay_ms()

#define PORT_LED PORTB
#define DDR_LED	 DDRB
#define PORT_LED_PIN PORTB6

#define DDR_QUAD DDRA
#define PORT_QUAD PORTA
#define PORT_QUAD_A PORTA0
#define PORT_QUAD_B PORTA1
#define PIN_QUAD PINA
#define PIN_QUAD_A PINA0
#define PIN_QUAD_B PINA1
#define INT_QUAD_A PCINT0
#define INT_QUAD_B PCINT1

static uint8_t pin_quad;

static int32_t counter;

int main()
{
	// Set led as output
	DDR_LED |= 1 << PORT_LED_PIN;

	int i;
	for (i = 0; i < 5; i++){
		PORT_LED |= 1 << PORT_LED_PIN;
		_delay_ms(100);
		PORT_LED &= ~(1 << PORT_LED_PIN);
		_delay_ms(100);
	}

	// Set quad pins as input
	DDR_QUAD &= ~(1 << PORT_QUAD_A);
	DDR_QUAD &= ~(1 << PORT_QUAD_B);
	DDR_QUAD &= ~(1 << PA2);
	DDR_QUAD &= ~(1 << PA3);
	// Set variables before interrupts
	pin_quad = PIN_QUAD;
	counter = 0;

	// Make sure no other pin change interrupt is enabled
	PCMSK0 = 0;
	PCMSK1 = 0;
	// Enable changes in pins of channel A and B
	PCMSK0 |= (1<<INT_QUAD_A);
	PCMSK0 |= (1<<INT_QUAD_B);		

	GIMSK |= (1<<PCIE1);
	// Enable general interrupts
	sei();
	
	// Turn on the led if the counter is positve
	while (1) {
		_delay_ms(50);
	}
}

// Interrupt routine for changes in pins 8:11
ISR(PCINT_vect){
	counter++;
	if (counter % 10 == 0){
		PORT_LED ^= 1 << PORT_LED_PIN;
	}
}
