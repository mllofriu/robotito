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


#define SLAVE_ADDR 0x0A   //this slave address (0x1, 0x2, 0x3)
#include "TinyWireS/utility/USI_TWI_Slave.h"

void receive_cb(uint8_t num_bytes)
{
  // if (num_bytes > 0){
  //   uint8_t b = usiTwiReceiveByte();
  //   // Most significant bit decides between
  //   // velocity commands and configuration ones
  //   if (b & 1 << 7) {
  //     // TODO: else receive configuration values
  //   } else {
  //     uint8_t vel = (b & 63);
  //     int8_t tics_per_cycle = ((int8_t)vel) - 32;
  //     // Second most significant bit switchs between motors
  //     if (b & 1 << 6) {
  //       m1.set_target(tics_per_cycle);
  //     } else {
  //       // TODO: other motor
  //     }
  //   }
  // }
  
}

void request_cb()
{
  usiTwiTransmitByte(1 & 255);
  usiTwiTransmitByte(1 >> 8);
  usiTwiTransmitByte(2 & 255);
  usiTwiTransmitByte(2 >> 8);
  usiTwiTransmitByte(3 & 255);
  usiTwiTransmitByte(3 >> 8);
  usiTwiTransmitByte(4 & 255);
  usiTwiTransmitByte(5 >> 8);

  // Write a control char to ensure proper sending
  usiTwiTransmitByte(0b01010101);
}

void enable_interrupts(){
	sei();
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
  // Enable HeartBeat
  DDRA |= 1 << PA3;
  
  // Setup communications
  usiTwiSlaveInit(10);
  usi_onReceiverPtr = receive_cb;
  usi_onRequestPtr = request_cb;

  enable_interrupts();

	while(1)
  {
    // TODO: put control in timer interrupt
    // int16_t pwm = m1.get_control_signal();
    // run_motor(pwm);

    // m1.new_control_cycle();

    TinyWireS_stop_check();

    // _delay_ms(control_period_ms);
  }
}
