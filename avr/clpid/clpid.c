#define F_CPU 8000000 // or whatever may be your frequency
 
#include <avr/io.h>
#include <util/delay.h>                // for _delay_ms()
#include <avr/interrupt.h>

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

#define INT_QUAD_A PCINT0
#define INT_QUAD_B PCINT1
#define PIN_QUAD PINA
#define PIN_QUAD_A PINA0
#define PIN_QUAD_B PINA1
#define PORT_QUAD_A PORTA0
#define PORT_QUAD_B PORTA1
#define DDR_QUAD DDRA

#define HB_DDR DDRB
#define HB_P PORTB
#define HB_N PORTB6

#define CONTROL_PERIOD_MS 100
#define CONTROL_PERIOD_S (.1)

#define DEFAULT_KP 20
#define DEFUALT_KTI .5

// The target tic counts for a control period 
static int16_t target;
// The tic counts for the current control period
static int16_t current;
// The proportional constant of the PI controller
static int16_t kp;
// The integral constant of the PI controller
static int16_t ki;
// Cached value of the target times the prop constant
static int16_t target_p;
// Cached value of the target times the integral constant
static int16_t target_i;
// Maximum allowed e * ki
static int16_t max_e_ki;
// The accumulated error times the prop constant
static int16_t pe;
// The accumulated error times the integral constant
static int16_t ie;
// The state of the encoder word
static uint8_t pin_quad, pin_quad_masked;


void set_target(int16_t tics_per_sec)
{
	int16_t tics_per_period = 
		tics_per_sec * CONTROL_PERIOD_S;
	target = tics_per_period;
	// TODO: overflow?
	target_p = target * kp;
	target_i = target * ki;
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


int main(void)
{
	// Set quad pins as input	
	DDR_QUAD &= ~(1 << PORT_QUAD_A);
	DDR_QUAD &= ~(1 << PORT_QUAD_B);
	//PORTA = 1 << PA2 | 1 << PA3;

	// Initialize variables
	target = 0;
	current = 0;
	pe = 0; ie = 0;
	target_p = 0; target_i = 0;
	max_e_ki = (int32_t) 256 * 128 - 1;
	pin_quad = PIN_QUAD;
	pin_quad_masked = pin_quad & 
		(1 << PIN_QUAD_A | 1 << PIN_QUAD_B);
	kp = DEFAULT_KP;
	ki = DEFUALT_KTI * DEFAULT_KP;
	// Enable the HearthBeat LED
	HB_DDR |= 1 << HB_N;
	
	int i;
	for (i = 0; i < 5; i++){
		HB_P |= 1 << HB_N;
		_delay_ms(100);
		HB_P &= ~(1 << HB_N);
		_delay_ms(100);
	}
	
	setup_motors();

	// TODO: Install interrupt to perform a control loop on timer


	// Set target to test
	int motor_red = 30;
	int gear_red = 1;
	int tics_per_turn = 12;
	int target_rps = 1;
	set_target(-motor_red * gear_red * tics_per_turn * target_rps);

	// Enable interrupt to handle encoder updates
	// Enable Interrupts for changes in pins PCINT[7:0] or PCINT[15:12]	
	GIMSK |= (1<<PCIE1);	
	// Enable the individual ports
	PCMSK0 = 0;
	PCMSK1 = 0;
	PCMSK0 |= (1<<INT_QUAD_A);
	PCMSK0 |= (1<<INT_QUAD_B);
	sei();

	M1_DIRA_P &= ~(1 << M1_DIRA_N);
	M1_DIRB_P |= (1 << M1_DIRB_N);
	//OCR1B = 150; 
	int iter = 0;
	while (1){
		
		// Use the last 8 bits for pwm - last bit is used for sign
		// TODO: overflow?
		int16_t pwm = pe / 128 + ie / 128;
		if (pwm < 0) {
			M1_DIRA_P &= ~(1 << M1_DIRA_N);
			M1_DIRB_P |= (1 << M1_DIRB_N);
			pwm = -pwm;
		} else {
			M1_DIRA_P |= (1 << M1_DIRA_N);
			M1_DIRB_P &= ~(1 << M1_DIRB_N);
		}
		OCR1B = pwm;
		// Reset pe to target * kp as the error is now target
		pe = target_p;
		// Add target * ki to the accumulated error * ki = ie
		// Watch for max_e_ki
		if (target_i > 0) {
			if (ie < max_e_ki - target_i)
				ie += target_i;
			else 
				ie = max_e_ki;
		} else if (target_i < 0) {
			if (ie > -max_e_ki - target_i)
				ie += target_i;
			else 
				ie = -max_e_ki;
		} 
		// Reset the current count of tics
		current = 0;
	
		iter ++;
		if (iter % 20 == 0) {
			target_rps = (target_rps + 1) % 8;
			set_target(-motor_red * gear_red * tics_per_turn * target_rps); 
		}

		_delay_ms(CONTROL_PERIOD_MS);
	}

}

int count=0;
// Interrupt routine for changes in pins 8:11
ISR(PCINT_vect){

	// http://www.me.unm.edu/~starr/teaching/me470/quad.pdf
	// Read new state
	uint8_t new_pin_quad = PIN_QUAD;

	uint8_t new_pin_quad_masked = new_pin_quad & 
		(1 << PIN_QUAD_A | 1 << PIN_QUAD_B);
	// Other unrelated pins might be triggering the interrupt
	// only perform computations if one of the pins is involved
	if (pin_quad_masked != new_pin_quad_masked) {
		// b_old ^ a_new == 1 iff dir is pos
		uint8_t b_old = pin_quad >> PIN_QUAD_B;
		uint8_t a_new = new_pin_quad >> PIN_QUAD_A;
		uint8_t dir = (a_new ^ b_old) & 1;
	
		if ((current < target && dir) || 
				(current > target && !dir)){
			pe -= kp;
			if (ie > -max_e_ki + ki)
				ie -= ki;
			else
				ie = -max_e_ki;
		} else if ((current >= target && dir) || 
				(current <= target && !dir)){
			pe += kp;
			if (ie < max_e_ki - ki)
				ie += ki;
			else
				ie = max_e_ki;
		}

		
		if (dir) {
			current++;
		} else {
			current--;
		}

		// Save new state as old
		pin_quad = new_pin_quad;
		pin_quad_masked = new_pin_quad_masked;

		count++;
		if (count % 100	 == 0)
			HB_P ^= 1 << HB_N;
	}

	
}
