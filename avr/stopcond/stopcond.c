#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#  define DDR_USI             DDRA
#  define PORT_USI            PORTA
#  define PIN_USI             PINA
#  define PORT_USI_SDA        PORTA6
#  define PORT_USI_SCL        PORTA4
#  define PIN_USI_SDA         PINA6
#  define PIN_USI_SCL         PINA4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_STR_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect

#  define DDR_LED 						DDRB
#  define PORT_LED						PORTB
#  define PIN_LED						  PORTB0


int main(){
	// Set up led as output
	DDR_LED |= 1 << PIN_LED;

	// Setup twi
	DDR_USI  |=  (1 << PORT_USI_SCL) | (1 << PORT_USI_SDA);
	PORT_USI |=  (1 << PORT_USI_SCL);
	PORT_USI |=  (1 << PORT_USI_SDA);
	DDR_USI  &= ~(1 << PORT_USI_SDA);
	USICR     =  (1 << USISIE) | (0 << USIOIE) | (1 << USIWM1) | (0 << USIWM0) |
		           (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | (0 << USITC);
	USISR     =  (1 << USI_START_COND_INT) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

	SREG = 1 << 7;

	// Blink before start
	int i;
	for (i = 0; i < 5; i++){
		PORT_LED = 1 << PIN_LED;
		_delay_ms(500);
		PORT_LED = 0 << PIN_LED;
		_delay_ms(500);
	}

	while (1){
	}

}

ISR(USI_START_VECTOR) {
	// Turn led upon receiving a stop cond
	PORT_LED = 1 << PIN_LED;
}
