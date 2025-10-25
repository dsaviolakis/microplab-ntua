#define F_CPU 16000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

#define PCA9555_0_ADDRESS 0x40 // A0=A1=A2=0 by hardware
#define TWI_READ 1 // reading from twi device
#define TWI_WRITE 0 // writing to twi device
#define SCL_CLOCK 100000L // twi clock in Hz

//Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2

// PCA9555 REGISTERS
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

#define LCD_RS  2
#define LCD_EN  3   

// Mask for data bits D4-D7 on PCA9555 OUTPUT_0
#define LCD_DATA_MASK 0xF0  // Data lines D4-D7 on bits 4-7

volatile uint8_t portd_state;

//----------- Master Transmitter/Receiver -------------------
#define TW_START 0x08
#define TW_REP_START 0x10

//---------------- Master Transmitter ----------------------
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28

//---------------- Master Receiver ----------------
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0X48
#define TW_MR_DATA_NACK 0x58

#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)

volatile uint16_t pressed_keys_tempo = 0x0000;
volatile uint16_t pressed_keys = 0x0000;

//initialize TWI clock
void twi_init(void)
{
	TWSR0 = 0; // PRESCALER_VALUE=1
	TWBR0 = TWBR0_VALUE; // SCL_CLOCK 100KHz
}

// Read one byte from the twi device (request more data from device)
unsigned char twi_readAck(void) 
{
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}

//Read one byte from the twi device, read is followed by a stop condition
unsigned char twi_readNak(void)
{
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}

// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
unsigned char twi_start(unsigned char address)
{
	uint8_t twi_status;
	
	// send START condition
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));

	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) return 1;

	// send device address
	TWDR0 = address;
	TWCR0 = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR0 & (1<<TWINT)));

	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK) ) {
	return 1;
	}

	return 0;
}

// Send start condition, address, transfer direction.
// Use ack polling to wait until device is ready
void twi_start_wait(unsigned char address) {
    uint8_t twi_status;
    while (1) {
	// send START condition
        TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

	// wait until transmission completed
        while (!(TWCR0 & (1 << TWINT)));

	// check value of TWI Status Register.
        twi_status = TW_STATUS & 0xF8;
        if (twi_status != TW_START && twi_status != TW_REP_START) continue;

	// send device address
        TWDR0 = address;
        TWCR0 = (1 << TWINT) | (1 << TWEN);

	// wail until transmission completed
        while (!(TWCR0 & (1 << TWINT)));

	// check value of TWI Status Register.
        twi_status = TW_STATUS & 0xF8;
        if (twi_status == TW_MT_SLA_NACK || twi_status == TW_MR_DATA_NACK) 
	{
	 /* device busy, send stop condition to terminate write operation */
            TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

	    // wait until stop condition is executed and bus released
            while (TWCR0 & (1 << TWSTO));
            continue;
        }
        break;
    }
}

// Send one byte to twi device, Return 0 if write successful or 1 if write failed
unsigned char twi_write( unsigned char data )
{
// send data to the previously addressed device
	TWDR0 = data;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	if( (TW_STATUS & 0xF8) != TW_MT_DATA_ACK) return 1;
	return 0;
}

// Send repeated start condition, address, transfer direction
//Return: 0 device accessible
// 	  1 failed to access device
unsigned char twi_rep_start(unsigned char address)
{
	return twi_start( address );
}

// Terminates the data transfer and releases the twi bus
void twi_stop(void)
{
	// send stop condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));
}

void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value)
{
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
}

uint8_t PCA9555_0_read(PCA9555_REGISTERS reg)
{
	uint8_t ret_val;
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
	ret_val = twi_readNak();
	twi_stop();
	return ret_val;
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

void Pwrite(uint8_t value){PCA9555_0_write(REG_OUTPUT_0,value);}

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

void lcd_command(uint8_t command) {
    portd_state &= ~(1 << LCD_RS); // RS = 0 for command
    write_2_nibbles(command);
    _delay_ms(250); // Wait for the command to process
}

void lcd_clear_display() {
    lcd_command(0x01); // Clear display command
    _delay_us(5); // Wait for clear command to process
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

void lcd_data(uint8_t data) {
    portd_state |= (1 << LCD_RS); 
    write_2_nibbles(data);
    _delay_us(250);
}

int main(void) {
    // Initialize TWI and PCA9555 I/O configuration
    twi_init();
    lcd_init();
    DDRB = 0xFF;
    PCA9555_0_write(REG_CONFIGURATION_0, 0x00); 
    PCA9555_0_write(REG_CONFIGURATION_1, 0xF0); 
    uint8_t p1_sol = '0', p2_sol = '9', p1,p2;
    
    while(1) { 
        PORTB = 0x00;
        lcd_clear_display();
        while((p1=keypad_to_ascii(0x01)) == 0x00){}
        lcd_data(p1);
        while((p2=keypad_to_ascii(0x01)) == 0x00){}
        lcd_data(p2);
        if(p1 == p1_sol && p2 == p2_sol){
            PORTB = 0xFF;
            _delay_ms(3000);
            PORTB = 0x00;
            _delay_ms(2000);
        }
        else{
            for(int i = 0; i < 5;i++){
            PORTB = 0xFF;
            _delay_ms(500);
            PORTB = 0x00;
            _delay_ms(500);
            }
        }
    }
}
