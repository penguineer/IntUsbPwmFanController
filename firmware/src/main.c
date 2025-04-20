#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define TWI_SLAVE_ADDRESS 0x42

/* TWI command "Get ID"
 *
 * This command returns the unique chip ID (4 bytes).
 */
#define TWI_CMD_GET_ID    0x01
/* TWI command "Set PWM"
 *
 * This command sets the target PWM value (1 byte).
 */
#define TWI_CMD_SET_PWM   0x02
/* TWI command "Get Tacho"
 *
 * This command returns the most recent completed tacho measurement (2 bytes).
 *
 * Please note that there is a delay between setting the PWM value and retrieving a matching tacho measurement.
 */
#define TWI_CMD_GET_TACHO 0x03

/***** Global variables *****/

uint8_t pwm_target = 0; // Target PWM value
uint16_t tacho_measurement = 0; // Current tacho measurement

/***** PWM  *****/

void update_target_pwm(const uint8_t) {
    // TODO implement
}


/***** TWI Core *****/

void TWI_slave_init(void) {
    // Configure SCL (PA2) and SDA (PA1) as inputs
    PORTA.DIRCLR = PIN2_bm | PIN1_bm; // leaves other pins unchanged

    // Enable internal pull-ups on SCL (PA2) and SDA (PA1)
    PORTA.PIN2CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN1CTRL |= PORT_PULLUPEN_bm;

    // Set the slave address
    TWI0.SADDR = TWI_SLAVE_ADDRESS << 1; // Shifted left by 1 for 7-bit addressing

    // Enable TWI Slave, enable stop interrupt, and enable data interrupt
    TWI0.SCTRLA = TWI_ENABLE_bm | TWI_PIEN_bm | TWI_DIEN_bm;

    // Clear any pending interrupts
    TWI0.SSTATUS = TWI_APIF_bm | TWI_DIF_bm;
}

#define TWI_BUFFER_SIZE 4

static uint8_t tx_buffer[TWI_BUFFER_SIZE]; // Buffer for data to send
static uint8_t tx_index = 0; // Current index in the buffer
static uint8_t tx_length = 0; // Number of bytes to send

void TWI_prepare_response(const uint8_t *data, const uint8_t length) {
    // Copy data to the transmission buffer
    tx_length = (length > TWI_BUFFER_SIZE) ? TWI_BUFFER_SIZE : length;
    // This loop is likely to be unrolled by the compiler
    for (uint8_t i = 0; i < tx_length; i++) {
        tx_buffer[i] = data[i];
    }

    // Reset index for new transmission
    tx_index = 0;
}

void TWI_handle_master_command(uint8_t, const uint8_t *, uint8_t);

#define RX_BUFFER_SIZE 2

static uint8_t rx_buffer[RX_BUFFER_SIZE]; // Buffer for received data
static uint8_t rx_index = 0; // Current index in the buffer

ISR(TWI0_TWIS_vect) {
    if (TWI0.SSTATUS & TWI_APIF_bm) {
        // Address or Stop condition interrupt
        if (TWI0.SSTATUS & TWI_AP_bm) {
            // Address match
            tx_index = 0; // Reset index on new transaction
        } else {
            // STOP condition detected
            if (rx_index > 0) {
                // Process received data
                TWI_handle_master_command(rx_buffer[0], &rx_buffer[1], rx_index + 1);
            }
            rx_index = 0; // Reset receive buffer index
        }
        TWI0.SSTATUS |= TWI_APIF_bm; // Clear interrupt flag
    }

    if (TWI0.SSTATUS & TWI_DIF_bm) {
        // Data interrupt
        if (TWI0.SSTATUS & TWI_DIR_bm) {
            // Master is reading (DIR = 1)
            if (tx_index < tx_length) {
                // Send next byte
                TWI0.SDATA = tx_buffer[tx_index++];
            } else {
                // Send NACK to indicate no more data
                TWI0.SCTRLB |= TWI_ACKACT_bm;
            }
        } else {
            // Master is writing (DIR = 0)
            if (rx_index < RX_BUFFER_SIZE) {
                // Store received data in the buffer
                rx_buffer[rx_index++] = TWI0.SDATA;
            } else {
                // Buffer full, send NACK
                TWI0.SCTRLB |= TWI_ACKACT_bm;
            }
        }
        TWI0.SSTATUS |= TWI_DIF_bm; // Clear interrupt flag
    }
}

/***** TWI Slave Communication *****/

uint32_t get_chip_id(void) {
    // Access the unique serial number from the SIGROW memory space
    const uint32_t chip_id = (uint32_t) SIGROW.SERNUM0 << 24 |
                             (uint32_t) SIGROW.SERNUM1 << 16 |
                             (uint32_t) SIGROW.SERNUM2 << 8 |
                             (uint32_t) SIGROW.SERNUM3;
    return chip_id;
}

void TWI_handle_master_command(const uint8_t command, const uint8_t *data, const uint8_t data_length) {
    switch (command) {
        case TWI_CMD_GET_ID:
            const uint32_t chip_id = get_chip_id();
            TWI_prepare_response((const uint8_t *) &chip_id, sizeof(chip_id));
            break;
        case TWI_CMD_SET_PWM:
            if (data_length == 1) {
                const uint8_t pwm_target = data[0];
                update_target_pwm(pwm_target);
            }
            break;
        case TWI_CMD_GET_TACHO:
            TWI_prepare_response((const uint8_t *) &tacho_measurement, sizeof(tacho_measurement));
            break;
        default:
            // no data
            TWI_prepare_response(NULL, 0);
            break;
    }
}

/***** Main *****/

int main(void) {
    TWI_slave_init();

    // ReSharper disable once CppDFAEndlessLoop
    for (;;);
}
