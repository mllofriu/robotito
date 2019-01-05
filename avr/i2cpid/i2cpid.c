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
#include "motor_controller.hpp"
#include "macros.h"

#define PWM_SET(ocr,val) _PWM_SET(ocr,val)
#define _PWM_SET(ocr,val) OCR1 ## ocr = val

#define CONTROL_PERIOD_S (.005)

#define DEFAULT_KP .016
#define DEFUALT_KTI 0.5
#define DEFAULT_MAX_E_KI 200

MotorController m1(
  DEFAULT_KP, DEFUALT_KTI * DEFAULT_KP, 
  DEFAULT_MAX_E_KI, CONTROL_PERIOD_S);
  
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
  SETUP_MOTOR(M1DIRA, M1DIRB, M1EN, M1TCCR, M1PWM);
  SETUP_MOTOR(M2DIRA, M2DIRB, M2EN, M2TCCR, M2PWM);
}

void enable_interrupts(){
	// Enable interrupt to handle encoder updates
	// Enable Interrupts for changes in pins PCINT[7:0] or PCINT[15:12]	
	GIMSK |= (1<<PCIE1);	
	// Enable the individual ports
	PCMSK0 = 0;
	PCMSK1 = 0;
	PCMSK0 |= (1<<INT_QUAD_A);
	PCMSK0 |= (1<<INT_QUAD_B);
	sei();
}

void run_motor(int16_t pwmval)
{
  if (pwmval < 0) {
    LOW(M1DIRA);
    HIGH(M1DIRB);
    pwmval = -pwmval;
  } else {
    LOW(M1DIRB);
    HIGH(M1DIRA);
  }

  PWM_SET(M1PWM,pwmval);
}

ISR(PCINT_vect){
  uint8_t pinquad = PIN_QUAD;
  bool a = (pinquad >> PIN_QUAD_A) & 1;
  bool b = (pinquad >> PIN_QUAD_B) & 1;
  m1.encoder_update(a, b);
}

int8_t sign(int16_t n)
{
  if (n < 0)
    return -1;
  else
    return 1;
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
  
  int motor_red = 30;
	int gear_red = 1;
	int tics_per_turn = 12;
	int target_rps = 2;
	m1.set_target(motor_red * gear_red * tics_per_turn * target_rps);

  enable_interrupts();

  int16_t control_period_ms = CONTROL_PERIOD_S * 1000;
	while(1)
  {
    // TODO: put control in timer interrupt
    int16_t pwm = m1.get_control_signal();
    // int16_t pwm_scaled = sign(pwm) *(200 + pwm / 255.0f * 50.f);
    run_motor(pwm);

    m1.new_control_cycle();

    TinyWireS_stop_check();

    _delay_ms(control_period_ms);
  }
}
