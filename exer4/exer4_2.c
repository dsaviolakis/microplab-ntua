#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LCD_RS PD2
#define LCD_EN PD3
#define LCD_DATA_MASK 0xF0

void setup() {
    DDRD = 0xFF;
    TCCR1A = (1 << WGM10) | (1 << COM1A1);
    TCCR1B = (1 << WGM12) | (1 << CS10);

    ADMUX = (1 << REFS0) | (1 << MUX0);    // Select ADC1 by setting MUX0 (ADC1 is channel 1)
    ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS2) | (1 << ADPS1);

    lcd_init();
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

void loop(){
    ADCSRA |= (1 << ADSC);                  
    while (ADCSRA & (1 << ADSC));           
    int adc_value = ADC * 5;
    int num = adc_value >> 10;
    int num1 = ((adc_value - (num << 10))*10) >> 10;
    int num2 = (((adc_value - (num << 10))*10 - (num1 << 10))*10 >> 10);
    num2 += 0b00110000;
    num1 += 0b00110000;
    num += 0b00110000;
    lcd_clear_display();
    lcd_data(num);
    lcd_data(0b00101110);
    lcd_data(num1);
    lcd_data(num2);
}

int main() {
    setup();
    while (1) {
        loop();
        _delay_ms(749);
    }
}
