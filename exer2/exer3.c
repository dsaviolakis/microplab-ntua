#define F_CPU 16000000UL   // Define CPU frequency for delay functions
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

volatile int counter = 0;

ISR(INT1_vect) // External INT1 ISR
{
    _delay_ms(5);
    if(counter>0){
        counter = 4700;   //not 5000 to account for other cycles in the execution
        PORTB = 0XFF; // Turn on all LEDs of PORTB
        _delay_ms(500);
    }
    else if(counter==0){
        counter=4700;
    }
    EIFR = (1 << INTF1);
}

int main(){
    // Interrupt on rising edge of INT1 pin
    EICRA=(1<<ISC11) | (1<<ISC10);
    EIMSK=(1<<INT1);
    sei(); // Enable global interrupts

    DDRB=0xFF; // Set PORTB as output
    PORTB =0x00;
    while(1){
        if(counter>0){
            PORTB = 0x01;
            _delay_ms(1);
            counter--;
        }

        if(counter==0){
            PORTB=0X00; // TURN OFF PB0
        }

    }
}
