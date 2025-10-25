#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LCD_RS PD2
#define LCD_EN PD3
#define LCD_DATA_MASK 0xF0

volatile int danger = 0;
volatile double conc = 0.0;

void setup() {
	DDRD = 0xFF; //Setup PORTD as output
	DDRB = 0xFF; //Setup PORTB as output

   	 // Configure Timer1 for Fast PWM mode, 8-bit, non-inverted
    TCCR1A = (1 << WGM10) | (1 << COM1A0);
   	TCCR1B = (1 << WGM12) | (1 << CS12); // Prescaler to 256

	ADMUX = (1 << REFS0) | (1 << MUX1);    // Select ADC2 by setting MUX0 (ADC1 is channel 1)
	ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS2) | (1 << ADPS1);

	TIMSK1 = (1 << TOIE1); //Enable timer1 interrupts

	//0.1 sec delay
	TCNT1 = 56285; //65535 - 6250

	sei();    //Enable global interrupts
	lcd_init(); //Initialize lcd screen
  	_delay_ms(100);
}


void lcd_init() {
    for (int i = 0; i < 3; i++) {
        PORTD = (PORTD & 0x0F) | 0x30; // Send 0x30 (8-bit mode)
        PORTD |= (1 << LCD_EN); // Enable pulse
        _delay_us(1);
        PORTD &= ~(1 << LCD_EN);
        _delay_us(250);
    }

    PORTD = (PORTD & 0x0F) | 0x20;
    PORTD |= (1 << LCD_EN);
    _delay_ms(1);
    PORTD &= ~(1 << LCD_EN);
    _delay_ms(250);

    lcd_command(0x28); // 4-bit mode, 2 lines, 5x8 font
    lcd_command(0x0C); // Display on, cursor off
    lcd_clear_display(); // Clear display
    lcd_command(0x06); // Increment cursor
}

void lcd_command(uint8_t command) {
    PORTD &= ~(1 << LCD_RS); // RS = 0 for command
    write_2_nibbles(command);
    _delay_ms(250); // Wait for the command to process
}

void lcd_clear_display() {
    lcd_command(0x01); // Clear display command
    _delay_us(5); // Wait for clear command to process
}


void write_2_nibbles(uint8_t data) {
    uint8_t temp = PORTD & 0x0F; // Preserve lower nibble of PORTD
    PORTD = (temp | (data & 0xF0)); // Send high nibble
    PORTD |= (1 << LCD_EN); // Enable pulse
    PORTD &= ~(1 << LCD_EN);

    PORTD = (temp | ((data << 4) & 0xF0)); // Send low nibble
    PORTD |= (1 << LCD_EN); // Enable pulse
    PORTD &= ~(1 << LCD_EN);
}

void lcd_data(uint8_t data) {
    PORTD |= (1 << LCD_RS);
    write_2_nibbles(data);
    _delay_us(250);
}

int main() {
	setup();

	while(1) {
		lcd_clear_display();
		if (danger) {
			lcd_data('G');
			lcd_data('A');
			lcd_data('S');
			lcd_data(' ');
			lcd_data('D');
			lcd_data('E');
			lcd_data('T');
			lcd_data('E');
			lcd_data('C');
			lcd_data('T');
			lcd_data('E');
			lcd_data('D');

			while(1) {
				PORTB = 0xFF;
				_delay_ms(500);
				PORTB = 0x00;
				_delay_ms(500);
				if (!danger)
					break;
			}

		} else {
			lcd_data('C');
			lcd_data('L');
			lcd_data('E');
			lcd_data('A');
			lcd_data('R');

			while(1) {
				if (conc < 10) {
					PORTB = 0x00;
               			} else if (conc < 20) {
                   			PORTB = 0x01;
				} else if (conc < 30) {
					PORTB = 0x03;
				} else if (conc < 40) {
					PORTB = 0x07;
				} else if (conc < 50) {
					PORTB = 0x0F;
				} else if (conc < 60) {
					PORTB = 0x1F;
				} else if (conc < 70) {
					PORTB = 0x3F;
				} else {
					PORTB = 0x7F;
				}
				if (danger)
					break;
			}
		}
	}
}



uint16_t read_adc() {
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC));  // Wait ending
    return ADC;
}

ISR(TIMER1_OVF_vect) {
	double adc_value = read_adc() * 1.0;

	double Vg = (adc_value * 5.0)/1024.0;
	conc = (Vg - 0.1) / 0.0129;

	if (conc > 70) {
		danger = 1;
	} else {
		danger = 0;
	}
	
    TCNT1 = 56285;
}
