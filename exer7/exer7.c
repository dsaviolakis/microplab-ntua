#define F_CPU 16000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

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

uint16_t get_data() {
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

// Function to print an integer to LCD (digit by digit)
void printInteger(int num) {
    // Handle the case where the number is 0
    if (num == 0) {
        lcd_data('0');
        return;
    }

    // Convert integer to string by dividing by 10 (starting from most significant digit)
    char digits[3]; // Up to 5 digits for integer part (e.g., 99999)
    int i = 0;
    while (num > 0) {
        digits[i++] = '0' + (num % 10); // Store each digit as character
        num /= 10;
    }

    // Print the digits in reverse order (most significant to least significant)
    for (int j = i - 1; j >= 0; j--) {
        lcd_data(digits[j]);
    }
}

void printDecimal(int num) {
    // If the number is negative, handle accordingly
    if (num < 0) {
        num = -num;  // Convert to positive for digit extraction
    }

    // Ensure we only consider the 3 most significant digits
    while (num >= 1000) {
        num /= 10;  // Keep dividing by 10 until the number is less than 1000
    }

    // Now, num contains the 3 most significant digits or fewer.
    // Extract each digit
    char first_digit = (num / 100) + '0';  // Extract the first digit
    char second_digit = ((num / 10) % 10) + '0';  // Extract the second digit
    char third_digit = (num % 10) + '0';  // Extract the third digit

    // Display each digit using lcd_data()
    lcd_data(first_digit);
    lcd_data(second_digit);
    lcd_data(third_digit);
}

int main() {
    //DDRD = 0xFF;
    twi_init();
    lcd_init();
    uint16_t temp_data;
    PCA9555_0_write(REG_CONFIGURATION_0, 0x00);
    while (1) {
        _delay_ms(1000);
        temp_data = get_data();
        if (temp_data == 0x8000) {
            lcd_clear_display();
            lcd_data('N');
            lcd_data('O');
            lcd_data(' ');
            lcd_data('D');
            lcd_data('e');
            lcd_data('v');
            lcd_data('i');
            lcd_data('c');
            lcd_data('e');
        } else {
            lcd_clear_display();
                        
            if (temp_data & 0x8000) {//Check if negative
                lcd_data('-');
                temp_data = ~temp_data + 1; // Convert full 16-bit value to positive
            } else {
                lcd_data('+');
            }

            uint8_t low_byte = temp_data & 0xFF;
            uint8_t high_byte = (temp_data >> 8) & 0xFF;

            
            int integer_part = 0;
            int fractional_part;
      
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

            
            printInteger(integer_part);
            lcd_data('.');
            printDecimal(fractional_part);
            lcd_data('"');
            lcd_data('C');
        }
    }
    return 0x01;
}

