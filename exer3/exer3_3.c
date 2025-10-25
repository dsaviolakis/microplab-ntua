#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int DC_VALUES[13] = {5, 25, 46, 66, 86, 107, 128, 147, 168, 188, 209, 229, 249};
int i = 6;
int dflag = 0;
int uflag = 0;
uint16_t mode_selector = 1;

void setup() {
    DDRB = 0xFF;                           // Set all bits of PORTB as output
    DDRD &= ~((1 << PD1) | (1 << PD2) | (1 << PD6) | (1 << PD7));    // Set PD6 and PD7 as input
    PORTD |= (1 << PD1) | (1 << PD2) | (1 << PD6) | (1 << PD7);      // Enable internal pull-up resistors for PD6 and PD7

    // Configure Timer1 for Fast PWM mode, 8-bit, non-inverted
    TCCR1A = (1 << WGM10) | (1 << COM1A1);
    TCCR1B = (1 << WGM12) | (1 << CS10);

    // Enable ADC0, ADC1 with AREF, internal VREF turned OFF
    ADMUX = (1 << REFS0);

    // Set Division Factor 128
    ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS2) | (1 << ADPS1);

    // Set initial duty cycle
    OCR1A = DC_VALUES[i];
    
    //Enable interrupts
    sei();
}

uint16_t read_adc() {
    ADCSRA |= (1 << ADSC); // Start conversion 
    while (ADCSRA & (1 << ADSC));  // Wait ending
    return ADC;
}

void mode1() {
    _delay_ms(5);
    // Check PD6 button press (decrease duty cycle)
    if (!(PIND & (1 << PD2))) {
        if (i > 0 && dflag == 0) {
            OCR1A = DC_VALUES[--i];
            dflag = 1;
        }
    }
    else{dflag = 0;}

    // Check PD7 button press (increase duty cycle)
    if (!(PIND & (1 << PD1))) {
        if (i < 12 && uflag == 0) {
            OCR1A = DC_VALUES[++i];
            uflag = 1;
        }
    }
    else{uflag = 0;}
}

void mode2() {
    uint16_t adc_value = read_adc();
    
    OCR1A = adc_value >> 2; //Convert to 8 bit
}

int main() {
    setup();
    while (1) {//Loop forever
        if (!(PIND & (1 << PD6))) {
            mode_selector = 1;
        } else if (!(PIND & (1 << PD7))) {
            mode_selector = 2;
        }

        if (mode_selector == 1) {
            mode1();
        } else {
            mode2();
        }
    }   
}

