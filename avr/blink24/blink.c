#define F_CPU 8000000 // or whatever may be your frequency
 
#include <avr/io.h>
#include <util/delay.h>                // for _delay_ms()
 
int main(void)
{
    DDRA = 0x01 << PA3;                       // initialize port C
    while(1)
    {
        // LED on
        PORTA = 1 << PA3;            // PC0 = High = Vcc
        _delay_ms(100);                // wait 500 milliseconds
 
        //LED off
        PORTA = 0;            // PC0 = Low = 0v
        _delay_ms(100);                // wait 500 milliseconds
    }
}
