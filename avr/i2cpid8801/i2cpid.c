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
#include "TinyWireS/utility/USI_TWI_Slave.h"

#include "pin_config.hpp"
#include "motor_controller.hpp"
#include "macros.h"

#define PWM_SET(ocr,val) _PWM_SET(ocr,val)
#define _PWM_SET(ocr,val) OCR1 ## ocr = val

constexpr float control_period_s = 0.005;
constexpr int16_t default_kp = .016f / control_period_s;
constexpr float default_kti = .5;
constexpr int16_t default_ki = default_kp * default_kti;
constexpr int16_t default_max_e_ki = 200;

MotorController m1(
  default_kp, default_ki, 
  default_max_e_ki);

// Tics per motor turn * motor gear reduction
// TODO: make this i2c configurable
constexpr int16_t tics_per_turn = 12 * 30;
constexpr float max_turns_per_sec_100 = 1100 / (60 * 100);
constexpr int target_rps = 0;
constexpr int target_init = tics_per_turn * target_rps * control_period_s;

void receive_cb(uint8_t num_bytes)
{
  if (num_bytes > 0){
    uint8_t b = usiTwiReceiveByte();
    // Most significant bit decides between
    // velocity commands and configuration ones
    if (b & 1 << 7) {
      // TODO: else receive configuration values
    } else {
      uint8_t vel = b & 127;
      int8_t tics_per_cycle = ((int8_t)vel) - 64;
      // Second most significant bit switchs between motors
      m1.set_target(tics_per_cycle);
    }
  }
  
}

void request_cb()
{
  int16_t accum_ticks = m1.get_accum_ticks();
  usiTwiTransmitByte(accum_ticks & 255);
  usiTwiTransmitByte(accum_ticks >> 8);
  int16_t accum_p_err = m1.get_accum_p_err();
  usiTwiTransmitByte(accum_p_err & 255);
  usiTwiTransmitByte(accum_p_err >> 8);
  int16_t accum_i_err = m1.get_accum_i_err();
  usiTwiTransmitByte(accum_i_err & 255);
  usiTwiTransmitByte(accum_i_err >> 8);
  int16_t ctrl_s = m1.get_last_control_signal();
  usiTwiTransmitByte(ctrl_s & 255);
  usiTwiTransmitByte(ctrl_s >> 8);

  // Write a control char to ensure proper sending
  usiTwiTransmitByte(0b01010101);
}

void setup_motors()
{
  SETUP_MOTOR(DIR, EN, TCCR, PWM);
}

void enable_interrupts(){
	// Enable interrupt to handle encoder updates
	// Enable Interrupts for changes in pins PCINT[11:8]	
	GIMSK |= (1<<PCIE1);	
	// Enable the individual ports
	PCMSK1 = 0;
	PCMSK1 |= (1<<INT_QUAD_A);
	PCMSK1 |= (1<<INT_QUAD_B);
	sei();
}

void run_motor(int16_t pwmval)
{
  if (pwmval < 0) {
    LOW(DIR);
    pwmval = -pwmval;
  } else {
    HIGH(DIR);
  }

  PWM_SET(PWM,pwmval);
}

ISR(PCINT1_vect){
  uint8_t pinquad = PIN_QUAD;
  bool a = (pinquad >> PIN_QUAD_A) & 1;
  bool b = (pinquad >> PIN_QUAD_B) & 1;
  m1.encoder_update(a, b);
  //TOGGLE(HBLED);
}

extern void    (*usi_onRequestPtr)(void);
extern void    (*usi_onReceiverPtr)(uint8_t);

void TinyWireS_stop_check() {
    if (!usi_onReceiverPtr) return;                         // no onReceive callback, nothing to do...
    if (!(USISR & (1 << USIPF))) return;                    // Stop not detected
    uint8_t amount = usiTwiAmountDataInReceiveBuffer();
    if (amount == 0) return;                                // no data in buffer
    usi_onReceiverPtr(amount);
}

int main()
{
  // Setup communications
  usiTwiSlaveInit(SLAVE_ADDR);
  usi_onReceiverPtr = receive_cb;
  usi_onRequestPtr = request_cb;

  // Enable HeartBeat
  OUTPUT(HBLED);

  // Read from the Fault line
  INPUT(FAULT);

  m1.set_target(target_init);

  // Enable PWM
  setup_motors();
  
  enable_interrupts();

  const int16_t control_period_ms = control_period_s * 1000;
  const int16_t hb_period_in_ctrl_tics = 1 / control_period_s;
  int16_t ctrl_tics_hb = 0;
	while(1)
  {
    // TODO: put control in timer interrupt
    int16_t pwm = m1.get_control_signal();
    run_motor(pwm);

    m1.new_control_cycle();

    TinyWireS_stop_check();

    if (!READ(FAULT)){
      // HIGH(HBLED);
    } else {
      ctrl_tics_hb++;
      if (ctrl_tics_hb % hb_period_in_ctrl_tics == 0) {
        TOGGLE(HBLED);
      }
    }

    _delay_ms(control_period_ms);
  }
}
