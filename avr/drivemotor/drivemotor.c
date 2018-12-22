#define F_CPU 1000000 // or whatever may be your frequency
 
#include <avr/io.h>
#include <util/delay.h>                // for _delay_ms()
 
#define M1_DIRA_P PORTA
#define M1_DIRA_N PA4
#define M1_DIRB_P PORTA
#define M1_DIRB_N PA5
#define M1_EN_P PORTB
#define M1_EN_N	PB3

#define M2_DIRA_P PORTA
#define M2_DIRA_N PA6
#define M2_DIRB_P PORTA
#define M2_DIRB_N PA7
#define M2_EN_P PORTB
#define M2_EN_N	PB5


#define HB_P PORTB
#define HB_N PORTB6

int main(void)
{
		DDRA = (1 << M1_DIRA_N) | (1 << M1_DIRB_N) | (1 << M2_DIRA_N) | (1 << M2_DIRB_N);
		DDRB |= 1 << M1_EN_N;
		DDRB |= 1 << M2_EN_N;

		M2_EN_P &= ~(1 << M2_EN_N);

    TCCR1A = (1 << COM1B1) | (0 << COM1B0) | (1 << PWM1B);
		TCCR1C |= (1 << COM1D1) | (0 << COM1D0) | (1 << PWM1D);
		TCCR1B = (1 << CS10);
    TCCR1D = (0 << WGM11) | (0 << WGM10);

		M1_DIRA_P |= (1 << M1_DIRA_N);
		M2_DIRA_P |= (1 << M2_DIRA_N);

		int vel = 0;
		int base_vel = 128;
		while(1)
    {
				HB_P = ~(HB_P & 1 << HB_N);
				vel += 32;
				OCR1B = base_vel + vel % (256 - base_vel)	;
				OCR1D = base_vel + vel % (256 - base_vel)	;
        _delay_ms(1000);
		}
}
