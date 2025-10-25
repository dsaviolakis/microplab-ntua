#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define PCA9555_0_ADDRESS 0x40
#define TWI_READ 1
#define TWI_WRITE 0
#define SCL_CLOCK 100000L

// TWI clock settings
#define TWBR0_VALUE ((F_CPU / SCL_CLOCK) - 16) / 2

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

int main(void) {
    // Initialize TWI and PCA9555 I/O configuration
    twi_init();
    PCA9555_0_write(REG_CONFIGURATION_0, 0xFC); // Set IO0_0 and IO0_1 as output, others as input

    // Loop forever
    while (1) {
        // Read inputs A, B, C, D from PORTB (lower 4 bits)
        uint8_t input = PINB & 0x0F; // Mask to keep only PB0-PB3
        

        // Extract individual bits for A, B, C, D
        uint8_t A = (input >> 0) & 1; // PORTB.0
        uint8_t B = (input >> 1) & 1; // PORTB.1
        uint8_t C = (input >> 2) & 1; // PORTB.2
        uint8_t D = (input >> 3) & 1; // PORTB.3

        // Calculate F0 = (A'BC + B'D)'
        uint8_t F0 = !((!A && B && C) || (!B && D));

        // Calculate F1 = (A + B + C) * (B * D')
        uint8_t F1 = (A || B || C) && (B && !D);

        // Write F0 to IO0_0 and F1 to IO0_1
        uint8_t output_value = (F0 << 0) | (F1 << 1); // IO0_0 = F0, IO0_1 = F1
        PCA9555_0_write(REG_OUTPUT_0, output_value);
    }
}
