#define F_CPU 16000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include <stdio.h>

#define PCA9555_0_ADDRESS 0x40
#define TWI_READ 1
#define TWI_WRITE 0
#define SCL_CLOCK 100000L
#define OFFSET 14

// TWI clock settings
#define TWBR0_VALUE (((F_CPU / SCL_CLOCK) - 16) / 2) // Corrected parentheses for proper calculation

// LCD control signals mapped to PCA9555 bits
#define LCD_RS  2
#define LCD_EN  3   

// Mask for data bits D4-D7 on PCA9555 OUTPUT_0
#define LCD_DATA_MASK 0xF0  // Data lines D4-D7 on bits 4-7

volatile uint16_t pressed_keys_tempo = 0x0000;
volatile uint16_t pressed_keys = 0x0000;

// PCA9555 Registers
typedef enum {
	REG_INPUT_0 = 0,
	REG_INPUT_1 = 1,
	REG_OUTPUT_0 = 2,
	REG_OUTPUT_1 = 3,
	REG_POLARITY_INV_0 = 4,
	REG_POLARITY_INV_1 = 5,
	REG_CONFIGURATION_0 = 6,
	REG_CONFIGURATION_1 = 7
} PCA9555_REGISTERS;

// TWI status codes
#define TW_START 0x08
#define TW_REP_START 0x10
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_NACK 0x58
#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)

volatile uint8_t portd_state;

// Initialize TWI clock
void twi_init(void) {
	TWSR0 = 0; // Prescaler value = 1
	TWBR0 = TWBR0_VALUE; // Set SCL to 100kHz
}

// Read one byte from the TWI device and request more data from the device
unsigned char twi_readAck(void) {
	TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR0 & (1 << TWINT)));
	return TWDR0;
}

// Read one byte from the TWI device and end transmission
unsigned char twi_readNak(void) {
	TWCR0 = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR0 & (1 << TWINT)));
	return TWDR0;
}

// Issues a start condition, sends address and transfer direction
// Returns 0 if device is accessible, 1 if failed
unsigned char twi_start(unsigned char address) {
	uint8_t twi_status;

	// Send START condition
	TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR0 & (1 << TWINT)));

	// Check the TWI status
	twi_status = TW_STATUS & 0xF8;
	if (twi_status != TW_START && twi_status != TW_REP_START) return 1;

	// Send device address
	TWDR0 = address;
	TWCR0 = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR0 & (1 << TWINT)));

	// Check the TWI status
	twi_status = TW_STATUS & 0xF8;
	if (twi_status != TW_MT_SLA_ACK && twi_status != TW_MR_SLA_ACK) {
		return 1;
	}
	return 0;
}

// Send start condition, address, transfer direction, with ack polling
void twi_start_wait(unsigned char address) {
	uint8_t twi_status;
	while (1) {
		TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
		while (!(TWCR0 & (1 << TWINT)));

		twi_status = TW_STATUS & 0xF8;
		if (twi_status != TW_START && twi_status != TW_REP_START) continue;

		TWDR0 = address;
		TWCR0 = (1 << TWINT) | (1 << TWEN);
		while (!(TWCR0 & (1 << TWINT)));

		twi_status = TW_STATUS & 0xF8;
		if (twi_status == TW_MT_SLA_NACK || twi_status == TW_MR_DATA_NACK) {
			TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
			while (TWCR0 & (1 << TWSTO));
			continue;
		}
		break;
	}
}

// Send one byte to the TWI device
// Returns 0 if write successful, 1 if write failed
unsigned char twi_write(unsigned char data) {
	TWDR0 = data;
	TWCR0 = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR0 & (1 << TWINT)));

	if ((TW_STATUS & 0xF8) != TW_MT_DATA_ACK) return 1;
	return 0;
}

// Send repeated start condition, address, transfer direction
unsigned char twi_rep_start(unsigned char address) {
	return twi_start(address);
}

// Terminates the data transfer and releases the TWI bus
void twi_stop(void) {
	TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	while (TWCR0 & (1 << TWSTO));
}

// Write to a PCA9555 register
void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value) {
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
}

// Read from a PCA9555 register
uint8_t PCA9555_0_read(PCA9555_REGISTERS reg) {
	uint8_t ret_val;
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
	ret_val = twi_readNak();
	twi_stop();
	return ret_val;
}

void Pwrite(uint8_t value) {
	PCA9555_0_write(REG_OUTPUT_0, value);
}

void write_2_nibbles(uint8_t data) {
	uint8_t temp = portd_state & 0x0F; // Preserve lower nibble of PORTD
	portd_state = (temp | (data & 0xF0)); // Send high nibble
	portd_state |= (1 << LCD_EN); // Enable pulse
	Pwrite(portd_state);
	portd_state &= ~(1 << LCD_EN);
	Pwrite(portd_state);
	portd_state = (temp | ((data << 4) & 0xF0)); // Send low nibble
	portd_state |= (1 << LCD_EN); // Enable pulse
	Pwrite(portd_state);
	portd_state &= ~(1 << LCD_EN);
	Pwrite(portd_state);
}

void lcd_command(uint8_t command) {
	portd_state &= ~(1 << LCD_RS); // RS = 0 for command
	write_2_nibbles(command);
	_delay_ms(250); // Wait for the command to process
}

void lcd_data(uint8_t data) {
	portd_state |= (1 << LCD_RS);
	write_2_nibbles(data);
	_delay_us(250);
}

void lcd_clear_display() {
	lcd_command(0x01); // Clear display command
	_delay_us(5); // Wait for clear command to process
}

void lcd_init() {
	portd_state = 0x00;
	for (int i = 0; i < 3; i++) {
		portd_state = (portd_state & 0x0F) | 0x30; // Send 0x30 (8-bit mode)
		Pwrite(portd_state);
		portd_state |= (1 << LCD_EN); // Enable pulse
		Pwrite(portd_state);
		_delay_us(1);
		portd_state &= ~(1 << LCD_EN);
		Pwrite(portd_state);
		_delay_us(250);
	}

	portd_state = (portd_state & 0x0F) | 0x20;
	portd_state |= (1 << LCD_EN);
	Pwrite(portd_state);
	_delay_ms(1);
	portd_state &= ~(1 << LCD_EN);
	Pwrite(portd_state);
	_delay_ms(250);

	lcd_command(0x28); // 4-bit mode, 2 lines, 5x8 font
	lcd_command(0x0C); // Display on, cursor off
	lcd_clear_display(); // Clear display
	lcd_command(0x06); // Increment cursor
}

uint8_t one_wire_reset(void) {
	// Set PD4 as output
	DDRD |= (1 << PD4);

	// Pull PD4 low to start reset pulse
	PORTD &= ~(1 << PD4);
	_delay_us(480); // Wait for 480 msec

	// Set PD4 as input and disable pull-up
	DDRD &= ~(1 << PD4);
	PORTD &= ~(1 << PD4);

	// Wait 100 microseconds for any connected devices to respond
	_delay_us(100);

	// Read the state of PD4
	uint8_t presence = PIND & (1 << PD4);

	// Wait another 380 microseconds (total reset time of 480 + 380 = 860us)
	_delay_us(380);

	// Check if any device pulled the line low (presence pulse)
	if (presence == 0) {
		return 1; // Device detected
	} else {
		return 0; // No device detected
	}
}

uint8_t one_wire_receive_bit(void) {
	// Set PD4 as output to initiate the time slot
	DDRD |= (1 << PD4);

	// Pull PD4 low to signal the start of the read time slot
	PORTD &= ~(1 << PD4);
	_delay_us(2); // Short delay of 2 microseconds

	// Set PD4 as input to release the line and allow the device to respond
	DDRD &= ~(1 << PD4);
	PORTD &= ~(1 << PD4); // Ensure pull-up is disabled

	// Wait 10 microseconds to allow the device time to drive the line
	_delay_us(10);

	// Read the state of PD4
	uint8_t bit = 0; // Default bit value to 0
	if (PIND & (1 << PD4)) {
		bit = 1; // If PD4 is high, set bit to 1
	}

	// Wait an additional 49 microseconds to complete the 1-Wire timing requirements
	_delay_us(49);

	// Return the read bit (0 or 1)
	return bit;
}

void one_wire_transmit_bit(uint8_t bit) {
	// Set PD4 as output to start the time slot
	DDRD |= (1 << PD4);

	// Pull PD4 low to initiate the time slot
	PORTD &= ~(1 << PD4);
	_delay_us(2); // Short delay of 2 microseconds

	// Decide whether to write a 1 or 0
	if (bit & 0x01) {
		// If bit is 1, release the line by setting PD4 high
		PORTD |= (1 << PD4);
	} else {
		// If bit is 0, keep the line low by clearing PD4
		PORTD &= ~(1 << PD4);
	}

	// Wait 58 microseconds to allow the device to sample the line
	_delay_us(58);

	// Set PD4 as input (release the line) and disable the pull-up
	DDRD &= ~(1 << PD4);
	PORTD &= ~(1 << PD4);

	// Recovery time to meet the 1-Wire protocol requirements
	_delay_us(1);
}

uint8_t one_wire_receive_byte(void) {
    uint8_t byte = 0; // Clear byte storage
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t bit = one_wire_receive_bit(); // Read a single bit
        byte >>= 1; // Shift byte to the right to make room for the new bit
        if (bit) {
            byte |= 0x80; // If the bit is 1, set the most significant bit
        }
    }
    return byte; // Return the received byte
}

void one_wire_transmit_byte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t bit = byte & 0x01; // Extract the least significant bit
        one_wire_transmit_bit(bit); // Transmit the bit
        byte >>= 1; // Shift the byte to the right for the next bit
    }
}

void usart_init(unsigned int ubrr){
	UCSR0A=0;
	UCSR0B=(1<<RXEN0)|(1<<TXEN0);
	UBRR0H=(unsigned char)(ubrr>>8);
	UBRR0L=(unsigned char)ubrr;
	UCSR0C=(3 << UCSZ00);
	return;
}

void usart_transmit(uint8_t data){
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0=data;
}

uint8_t usart_receive(){
	while(!(UCSR0A&(1<<RXC0)));
	return UDR0;
}

uint8_t scan_row(int row){
    int param = 0;
    if(row == 1){param = 0x07;}
    else if(row == 2){param = 0x0B;}
    else if(row == 3){param = 0x0D;}
    else if(row == 4){param = 0x0E;}
    if(param != 0){
        PCA9555_0_write(REG_OUTPUT_1, param); 
        _delay_us(100);
        return ~PCA9555_0_read(REG_INPUT_1) >> 4;
    }
    return 0x00;
}

uint16_t scan_keypad(){
    uint16_t res = 0x00;
    uint8_t par;
    par = scan_row(1);
    res += par & 0x000F;
    par = scan_row(2);
    res += (par << 4) & 0x00F0;
    par = scan_row(3);
    res += (par << 8) & 0x0F00;
    par = scan_row(4);
    res += (par << 12) & 0xF000;
    return res;
}

uint16_t scan_keypad_rising_edge(){
    pressed_keys_tempo = scan_keypad();
    _delay_ms(10);
    uint16_t temp = scan_keypad();
    pressed_keys_tempo = pressed_keys_tempo & temp;
    uint16_t res = pressed_keys_tempo & ~pressed_keys;
    pressed_keys = pressed_keys_tempo;
    return res;
}

uint8_t keypad_to_ascii(uint8_t flag){
    uint16_t keys = 0x0000;
    if(flag == 0x00){keys = scan_keypad();}
    else if(flag == 0x01){keys = scan_keypad_rising_edge();}
    else return 0x00;
    int counter = 0;
    while((keys & 0x0001)==0x0000 && (counter < 17)){
        counter++;
        keys = keys >> 1;
    }
    switch(counter){
        case 0:
            return '1';
        case 1:
            return '2';
        case 2:
            return '3';
        case 3:
            return 'A';
        case 4:
            return '4';
        case 5:
            return '5';
        case 6:
            return '6';
        case 7:
            return 'B';
        case 8:
            return '7';
        case 9:
            return '8';
        case 10:
            return '9';
        case 11:
	    return 'C';
        case 12:
            return '*';
        case 13:
            return '0';
        case 14:
            return '#';
        case 15:
            return 'D';
        default :
            return 0;
    }
}

void setup() {
	usart_init(103);
	_delay_ms(10);
	DDRD = 0xFF; 
	twi_init();
	_delay_ms(10);
	lcd_init();
	_delay_ms(10);
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00); 
	_delay_ms(10);
	PCA9555_0_write(REG_CONFIGURATION_1, 0xF0);
	_delay_ms(10);

    	TCCR1A = (1 << WGM10) | (1 << COM1A1);
	TCCR1B = (1 << WGM12) | (1 << CS10);

	ADMUX = (1 << REFS0) | (0 << MUX0);    // Select ADC1 by setting MUX0 (ADC1 is channel 1)
	ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS2) | (1 << ADPS1);
    	return;
}

void esp_restart() {
	usart_transmit('E');
	usart_transmit('S');
	usart_transmit('P');
	usart_transmit(':');
	usart_transmit('r');
	usart_transmit('e');
	usart_transmit('s');
	usart_transmit('t');
	usart_transmit('a');
	usart_transmit('r');
	usart_transmit('t');
	usart_transmit('\n');
}

void esp_connect() {
	usart_transmit('E');
	usart_transmit('S');
	usart_transmit('P');
	usart_transmit(':');
	usart_transmit('c');
	usart_transmit('o');
	usart_transmit('n');
	usart_transmit('n');
	usart_transmit('e');
	usart_transmit('c');
	usart_transmit('t');
	usart_transmit('\n');
}

void esp_send_url() {
    usart_transmit('E');
    usart_transmit('S');
    usart_transmit('P');
    usart_transmit(':');
    usart_transmit('u');
    usart_transmit('r');
    usart_transmit('l');
    usart_transmit(':');
    usart_transmit('"');
    usart_transmit('h');
    usart_transmit('t');
    usart_transmit('t');
    usart_transmit('p');
    usart_transmit(':');
    usart_transmit('/');
    usart_transmit('/');
    usart_transmit('1');
    usart_transmit('9');
    usart_transmit('2');
    usart_transmit('.');
    usart_transmit('1');
    usart_transmit('6');
    usart_transmit('8');
    usart_transmit('.');
    usart_transmit('1');
    usart_transmit('.');
    usart_transmit('2');
    usart_transmit('5');
    usart_transmit('0');
    usart_transmit(':');
    usart_transmit('5');
    usart_transmit('0');
    usart_transmit('0');
    usart_transmit('0');
    usart_transmit('/');
    usart_transmit('d');
    usart_transmit('a');
    usart_transmit('t');
    usart_transmit('a');
    usart_transmit('"');
    usart_transmit('\n');
    return;
}

uint16_t get_temp_data() {
    if (!one_wire_reset()) {
        return 0x8000;
    }

    one_wire_transmit_byte(0xCC); //Only one device already

    one_wire_transmit_byte(0x44); //Init a temp measurement

    while (!one_wire_receive_byte()); //Wait until measurement is finished (one_wire_receive_bit==1)

    if (!one_wire_reset()) {
        return 0x8000;
    }

    one_wire_transmit_byte(0xCC);

    one_wire_transmit_byte(0xBE); //Read 16bit temp data

    uint8_t low_byte = one_wire_receive_byte(); //Receive temp data as 16bit number

    uint8_t high_byte = one_wire_receive_byte();

    int16_t temperature_data = ((int16_t) high_byte << 8) | low_byte;
    return temperature_data;
}

float process_temp_data() {
	uint16_t temp_data = get_temp_data();
	if (temp_data == 0x8000) {
		return -1.0;
	} else {
		uint8_t low_byte = temp_data & 0xFF;
		uint8_t high_byte = (temp_data >> 8) & 0xFF;

		int integer_part = OFFSET;
		int fractional_part = 0;

		if (low_byte & 0b00000001) fractional_part += 625;
		if (low_byte & 0b00000010) fractional_part += 1250;
		if (low_byte & 0b00000100) fractional_part += 2500;
		if (low_byte & 0b00001000) fractional_part += 5000;
		if (low_byte & 0b00010000) integer_part += 1;
		if (low_byte & 0b00100000) integer_part += 2;
		if (low_byte & 0b01000000) integer_part += 4;
		if (low_byte & 0b10000000) integer_part += 8;
		if (high_byte & 0b00000001) integer_part += 16;
		if (high_byte & 0b00000010) integer_part += 32;
		if (high_byte & 0b00000100) integer_part += 64;

		float temp = 0.0;
		temp += integer_part;
		temp += (float)fractional_part / 10000;

		return temp;
	}
}

int update_status(float pressure, float temperature, int status) {
	uint8_t key;

	if (status == 1) { //Status  NURSE CALL
		key = keypad_to_ascii(0);
		if(key == '#') {
			uint8_t counter = 0;
			while (counter < 20) {
				_delay_ms(50);
				if(keypad_to_ascii(0) != '#') return status;
				counter++;
			}
			if (pressure > 12 || pressure < 4) {
				status = 2; //Change status to CHECK PRESSURE
			} else if (temperature < 34 || temperature > 37) {
				status = 3; //Change status to CHECK TEMP
			} else {
				status = 0; //Change status to OK
			}
		}
		return status;
	} else {
		key = keypad_to_ascii(1);
		if(key == '9') {//9 is Pressed
			status = 1; //Change status to NURSE CALL
		} else if (pressure > 12 || pressure < 4) {
			status = 2; //Change status to CHECK PRESSURE
		} else if (temperature < 34 || temperature > 37) {
			status = 3; //Change status to CHECK TEMP
		} else {
			status = 0;
		}
		return status;
	}

}

char* get_status(int status_num) {
	char* status;

	if (status_num == 0) {
		status = "OK";
	} else if (status_num == 1) {
		status = "NURSE CALL";
	} else if (status_num == 2) {
		status = "CHECK PRESSURE";
	} else if (status_num == 3) {
		status = "CHECK TEMP";
	} else {
       		status = "UNKNOWN";
	}

	return status;
}

float process_pressure_data() {
	// Start ADC conversion
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));  // Wait for conversion to complete

	// Read and scale ADC value
	int adc_value = ADC; // Assuming ADC is configured for 10-bit resolution
	float scaled_pressure = (adc_value / 1023.0) * 20.0; // Map to 0-20 range

	// Return the pressure as a float
	return scaled_pressure;
}

void print_float(float f) {
	if (f == -1) {
		lcd_data('-');
		lcd_data('1');
		lcd_data('.');
		lcd_data('0');
		return;
	}
	
	// Convert scaled_pressure to integer parts
	int whole_part = (int)f; // Get the integer part
	int fractional_part = (int)((f - whole_part) * 10); // Get one decimal place
									  
	// Extract digits for display
	int digit1 = whole_part / 10; // Tens place of whole part
	int digit2 = whole_part % 10; // Units place of whole part
	
	
		
	lcd_data(digit1 + '0');
	lcd_data(digit2 + '0');
	lcd_data('.');
	lcd_data(fractional_part + '0');

	return;
}

void print_string(char *str) {
	for(int i=0; str[i] != '\0'; i++)
		lcd_data(str[i]);
}

void send_payload(float temperature, float pressure, int status_num) {
	char* status;
	char payload[512];

	status = get_status(status_num);
	char temp_str[8], press_str[8];

	//Convert temp and pressure to strings
	snprintf(temp_str, sizeof(temp_str), "%d.%d", (int)temperature, (int)((temperature - (int)temperature) * 10));
	snprintf(press_str, sizeof(press_str), "%d.%d", (int)pressure, (int)((pressure - (int)pressure) * 10));

	snprintf(payload, sizeof(payload),
        "ESP:payload:[{\"name\":\"temperature\",\"value\":\"%s\"},"
        "{\"name\":\"pressure\",\"value\":\"%s\"},"
        "{\"name\":\"team\",\"value\":\"9\"},"
        "{\"name\":\"status\",\"value\":\"%s\"}]\n",
        temp_str, press_str, status);

	for (int i = 0; payload[i] != '\0'; i++) {
		usart_transmit(payload[i]);
	}

}

void esp_send_transmit() {
	const char* command = "ESP:transmit\n";
	for (int i = 0; command[i] != '\0'; i++) {
		usart_transmit(command[i]);
	} 
}

int main() {
	setup();
    char resp[30];
	//esp_restart();
    
	int j = 0;
	char x = '0';
    esp_connect();
	    while(x!='\n' || j == 0) {
        x = usart_receive();
        if(('0'<=x && x<='9') || ('a'<=x && x<'z') || ('A'<=x && x<='Z') || x==' '){resp[j++]=x;}
    }
    lcd_data('1');
    lcd_data('.');
    for(int i = 0; i < j; i++) lcd_data( resp[i]);
    _delay_ms(2000);
    j=0;
    esp_send_url();
    while(x!='\n' || j == 0) {
        x = usart_receive();
        if(('0'<=x && x<='9') || ('a'<=x && x<='z') || ('A'<=x && x<='Z') || x==' '){resp[j++]=x;}
    }
    lcd_clear_display();
    lcd_data('2');
    lcd_data('.');
    for(int i = 0; i < j; i++) lcd_data( resp[i]);
	_delay_ms(2000);

	float temperature;
	float pressure;
	int status_num = 0;
	while(1) {
		//Get data
		pressure = process_pressure_data();
		temperature = process_temp_data();
		status_num = update_status(pressure, temperature, status_num);

		//Print data
		lcd_clear_display();
		print_string("TMP:");
		print_float(temperature);
		print_string(" PR:");
		print_float(pressure);

		char *status = get_status(status_num);
		
		for(int i=0; i<24; i++) {
			lcd_data(' ');
		}

		print_string(status);
		for(long int q = 0;q<55;q++){
           		 _delay_ms(10);
           		 status_num = update_status(pressure, temperature, status_num);
        	}        
		send_payload(temperature, pressure, status_num);

		int i = 0;
		char x = '0';
		while(x!='\n' || i==0) {
			x = usart_receive();
			if(('0'<=x && x<='9') || ('a'<=x && x<='z') || ('A'<=x && x<='Z') || x==' '){
	    			resp[i++]=x;
			}
			
		}
		lcd_clear_display();
    	lcd_data('3');
    	lcd_data('.');
   		for(int j = 0; j < i; j++) lcd_data( resp[j]);
		for(long int q = 0;q<55;q++){
            _delay_ms(10);
            status_num = update_status(pressure, temperature, status_num);
        }
		esp_send_transmit();

        
        i = 0;
		x = '0';
		while(x!='\n' || i==0) {
			x = usart_receive();
			if(('0'<=x && x<='9') || ('a'<=x && x<='z') || ('A'<=x && x<='Z') || x==' '){
	    			resp[i++]=x;
			}
			
		}
		lcd_clear_display();
        lcd_data('4');
    	lcd_data('.');
		for(int j = 0; j < i; j++) lcd_data( resp[j]);
        for(long int q = 0;q<55;q++){
            _delay_ms(10);
            status_num = update_status(pressure, temperature, status_num);
        }
		
	}

	return 0;
}
