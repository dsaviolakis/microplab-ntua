#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int DC_VALUES[13] = {5, 25, 46, 66, 86, 107, 128, 147, 168, 188, 209, 229, 249};
int index = 6;
int dflag = 0;
int uflag = 0;
volatile uint16_t adc_sum = 0;
volatile uint8_t adc_count = 0;
volatile uint16_t adc_avg = 0;

void setup() {
    DDRB |= (1 << PB1);                    // Set OC1A (PB1) as output
    DDRB = 0xFF;                           // Set all bits of PORTB as output
    DDRD &= ~((1 << PD6) | (1 << PD7));    // Set PD6 and PD7 as input
    PORTD |= (1 << PD6) | (1 << PD7);      // Enable internal pull-up resistors for PD6 and PD7

    // Configure Timer1 for Fast PWM mode, 8-bit, non-inverted
    TCCR1A = (1 << WGM10) | (1 << COM1A1);
    TCCR1B = (1 << WGM12) | (1 << CS10);

    // Set initial duty cycle
    OCR1A = DC_VALUES[index];
}

void loop() {
    _delay_ms(5);
    // Check PD6 button press (decrease duty cycle)
    if (!(PIND & (1 << PD6))) {
        if (index > 0 && dflag == 0) {
            OCR1A = DC_VALUES[--index];
            dflag = 1;
        }
    }
    else{dflag = 0;}

    // Check PD7 button press (increase duty cycle)
    if (!(PIND & (1 << PD7))) {
        if (index < 12 && uflag == 0) {
            OCR1A = DC_VALUES[++index];
            uflag = 1;
        }
    }
    else{uflag = 0;}
}

int main() {
    setup();
    while (1) {
        loop();
    }
}
