#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

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

void show_names(char Names[]){
    lcd_clear_display();
    for(int i=0;i<16;i++){
        lcd_data(Names[i]);
    }
    _delay_ms(2000);
    for(int i = 1;i < 34;i++){
        lcd_clear_display();
        for(int j = i;j<i+16;j++){lcd_data(Names[j]);}
        _delay_ms(750);
    }
    _delay_ms(2000);
}

void show_names_2_rows(char Names[]){
    lcd_clear_display();
    for(int i=0;i<3;i++){
        lcd_data(' ');
    }
    for(int i=0;i<9;i++){
        lcd_data(Names[i]);
    }
    for(int i=0;i<31;i++){
        lcd_data(' ');
    }
    for(int i=10;i<20;i++){
        lcd_data(Names[i]);
    }
    _delay_ms(3000);
    lcd_clear_display();
    for(int i=0;i<3;i++){
        lcd_data(' ');
    }
    for(int i=0;i<9;i++){
        lcd_data(Names[i]);
    }
    for(int i=0;i<28;i++){
        lcd_data(' ');
    }
    for(int i=34;i<49;i++){
        lcd_data(Names[i]);
    }
    _delay_ms(3000);
}

int main(void) {
    DDRD = 0xFF;
    twi_init();
    PCA9555_0_write(REG_CONFIGURATION_0, 0x00); // Set IO0_0 and IO0_1 as output, others as input

    lcd_init();
    char Names[49];

    Names[0] = 'D';
    Names[1] = 'I';
    Names[2] = 'M';
    Names[3] = 'I';
    Names[4] = 'T';
    Names[5] = 'R';
    Names[6] = 'I';
    Names[7] = 'O';
    Names[8] = 'S';
    Names[9] = ' ';
    Names[10] = 'S';
    Names[11] = 'A';
    Names[12] = 'V';
    Names[13] = 'I';
    Names[14] = 'O';
    Names[15] = 'L';
    Names[16] = 'A';
    Names[17] = 'K';
    Names[18] = 'I';
    Names[19] = 'S';
    Names[20] = ' ';
    Names[21] = '&';
    Names[22] = '&';
    Names[23] = ' ';
    Names[24] = 'D';
    Names[25] = 'I';
    Names[26] = 'M';
    Names[27] = 'I';
    Names[28] = 'T';
    Names[29] = 'R';
    Names[30] = 'I';
    Names[31] = 'O';
    Names[32] = 'S';
    Names[33] = ' ';
    Names[34] = 'D';
    Names[35] = 'I';
    Names[36] = 'M';
    Names[37] = 'I';
    Names[38] = 'T';
    Names[39] = 'R';
    Names[40] = 'A';
    Names[41] = 'K';
    Names[42] = 'O';
    Names[43] = 'P';
    Names[44] = 'O';
    Names[45] = 'U';
    Names[46] = 'L';
    Names[47] = 'O';
    Names[48] = 'S';

    // Main loop
    while (1) {
        show_names(Names);
        show_names_2_rows(Names);
    }
}
