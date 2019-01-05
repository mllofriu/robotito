#pragma once

#define HB_DDR DDRB
#define HB_P PORTB
#define HB_N PORTB6

#define M1DIRA A,4
#define M1DIRB A,5
#define M1EN B,3
#define M1TCCR A
#define M1PWM B

#define M2DIRA A,6
#define M2DIRB A,7
#define M2EN B,5
#define M2TCCR C
#define M2PWM D

#define INT_QUAD_A PCINT0
#define INT_QUAD_B PCINT1
#define PIN_QUAD PINA
#define PIN_QUAD_A PINA0
#define PIN_QUAD_B PINA1
#define PORT_QUAD_A PORTA0
#define PORT_QUAD_B PORTA1
#define DDR_QUAD DDRA