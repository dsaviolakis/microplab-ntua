#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int DC_VALUES[13] = {5, 25, 46, 66, 86, 107, 128, 147, 168, 188, 209, 229, 249};
int index = 6;
int dflag = 0;
int uflag = 0;
int volt_sum = 0;
int volt_count = 0;

void setup() {
    DDRB |= (1 << PB1);
    DDRB = 0xFF;
    DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    DDRD &= ~((1 << PD6) | (1 << PD7));    
    PORTD |= (1 << PD6) | (1 << PD7);      

    TCCR1A = (1 << WGM10) | (1 << COM1A1);
    TCCR1B = (1 << WGM12) | (1 << CS10);
    OCR1A = DC_VALUES[index];

    ADMUX = (1 << REFS0) | (1 << MUX0);
    ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS2) | (1 << ADPS1);

    TCCR2A = 0;  
    TCCR2B = (1 << CS22) | (1 << CS20);
    TIMSK2 = (1 << TOIE2);  

    sei();  
}

void control_leds() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    int adc_value = ADC;
    
    if(++volt_count < 16){
        volt_sum += adc_value;
    }
    else{
        volt_sum += adc_value;
        adc_value = volt_sum >> 4;
        PORTD &= ~((1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD4));

        if (adc_value <= 200) {
            PORTD |= (1 << PD0);
        } else if (adc_value <= 400) {
            PORTD |= (1 << PD1);
        } else if (adc_value <= 600) {
            PORTD |= (1 << PD2);
        } else if (adc_value <= 800) {
            PORTD |= (1 << PD3);
        } else {
            PORTD |= (1 << PD4);
        }
        volt_sum = 0;
        volt_count = 0;
    }
}

ISR(TIMER2_OVF_vect) {
    static uint16_t overflow_count = 0;
    if (++overflow_count >= 61) {
        overflow_count = 0;
        control_leds();
    }
}

void loop() {
    if (!(PIND & (1 << PD6))) {
        if (index > 0 && dflag == 0) {
            OCR1A = DC_VALUES[--index];
            dflag = 1;
        }
    } else {
        dflag = 0;
    }

    if (!(PIND & (1 << PD7))) {
        if (index < 12 && uflag == 0) {
            OCR1A = DC_VALUES[++index];
            uflag = 1;
        }
    } else {
        uflag = 0;
    }
}

int main() {
    setup();
    while (1) {
        loop();
    }
}
