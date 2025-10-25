#define F_CPU 16000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include <string.h>

#define PCA9555_0_ADDRESS 0x40
#define TWI_READ 1
#define TWI_WRITE 0
#define SCL_CLOCK 100000L

// TWI clock settings
#define TWBR0_VALUE (((F_CPU / SCL_CLOCK) - 16) / 2) // Corrected parentheses for proper calculation

// LCD control signals mapped to PCA9555 bits
#define LCD_RS  2
#define LCD_EN  3   

// Mask for data bits D4-D7 on PCA9555 OUTPUT_0
#define LCD_DATA_MASK 0xF0  // Data lines D4-D7 on bits 4-7

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

void usart_send_string(const char *str) {
    while (*str) {
        usart_transmit(*str++);
    }
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

void get_usart_response(char *resp){
    for(int i =0;i<30;i++){
        resp[i] = usart_receive();
        if (resp[i] == '\n') break;
    }
}

uint8_t compare_response(char *resp,char *comp,int size){
    for(int i = 0;i<size;i++){
        if(comp[i]!=resp[i]){return 0x01;}
    }
    return 0x00;
}

void esp_connect(){
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
    return;
}

void esp_restart(){
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
    return;
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


void setup(){
    usart_init(103);
    _delay_ms(10);
    DDRD = 0xFF; 
    twi_init();
    _delay_ms(10);
    lcd_init();
    _delay_ms(10);
    PCA9555_0_write(REG_CONFIGURATION_0, 0x00); 
    _delay_ms(10);
    return;
}

int main(void) {
    char resp[300];
    for(int i = 0;i<30;i++){resp[i]=0;}
    int j = 0;
    char x = '0';
    setup(); //Set up configuration
    lcd_clear_display();
    esp_connect();
    while(x!='\n' || j == 0) { //Keep only alphabetical characters
        x = usart_receive();
        if(('0'<=x && x<='9') || ('a'<=x && x<'z') || ('A'<=x && x<='Z')){resp[j++]=x;}
    }
    lcd_data('1');
    lcd_data('.');
    for(int i = 0; i < j; i++) lcd_data( resp[i]); //Print answer
    _delay_ms(2000);
    j=0;
    esp_send_url(); /Set up url
    while(x!='\n' || j == 0) { //Keep only alphabetical characters
        x = usart_receive();
        if(('0'<=x && x<='9') || ('a'<=x && x<='z') || ('A'<=x && x<='Z')){resp[j++]=x;}
    }
    lcd_clear_display();
    lcd_data('2');
    lcd_data('.');
    for(int i = 0; i < j; i++) lcd_data( resp[i]); //Print answer
    return 0;
}
