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

// Tics per motor turn * motor gear reduction
// TODO: make this i2c configurable
const int16_t tics_per_turn = 12 * 30;
const float max_turns_per_sec_100 = 1100 / (60 * 100);

void receive_cb(uint8_t num_bytes)
{
  if (num_bytes > 0){
    uint8_t b = TinyWireS.read();
    // Most significant bit decides between
    // velocity commands and configuration ones
    if (b & 1 << 7) {
      // TODO: else receive configuration values
    } else {
      uint8_t vel = (b & 63);
      int8_t tics_per_cycle = ((int8_t)vel) - 32;
      // Second most significant bit switchs between motors
      if (b & 1 << 6) {
        m1.set_target(tics_per_cycle);
      } else {
        // TODO: other motor
      }
    }
  }
  
}

void request_cb()
{
  int16_t accum_ticks = m1.get_accum_ticks();
  TinyWireS.write(accum_ticks & 255);
  TinyWireS.write(accum_ticks >> 8);
  int16_t accum_p_err = m1.get_accum_p_err();
  TinyWireS.write(accum_p_err & 255);
  TinyWireS.write(accum_p_err >> 8);
  int16_t accum_i_err = m1.get_accum_i_err();
  TinyWireS.write(accum_i_err & 255);
  TinyWireS.write(accum_i_err >> 8);
  int16_t ctrl_s = m1.get_last_control_signal();
  TinyWireS.write(ctrl_s & 255);
  TinyWireS.write(ctrl_s >> 8);

  // Write a control char to ensure proper sending
  TinyWireS.write(0b01010101);
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
  
	int target_rps = 2;
	m1.set_target(tics_per_turn * target_rps * CONTROL_PERIOD_S);

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
