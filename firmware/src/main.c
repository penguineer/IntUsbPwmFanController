#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

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
uint16_t tacho_rpm = 0; // Most recent tacho measurement

/***** PWM *****/

// Assumes a prescaler of 8 and a frequency of 25 kHz
#define PWM_PREBUF ((F_CPU / (8 * 25000)) - 1)

void TCA_init_pwm(void) {
    // Configure PA3 as output
    PORTA.DIRSET = PIN3_bm; // Set PA3 as output

    // TCA0 WO[1] routes to PA3

    // Set TCA0 to single-slope PWM mode
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm; // Enable WO1 (PA3)

    // Set the period
    TCA0.SINGLE.PER = PWM_PREBUF;

    // Set the compare value for 0% duty cycle
    TCA0.SINGLE.CMP1 = 0; // 0% duty cycle

    // Enable the TCA0 clock with a prescaler of 8
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
}

void update_target_pwm(const uint8_t duty_cycle_param) {
    // Store the setting
    pwm_target = duty_cycle_param;

    // Perform the calculation in a 32-bit space to avoid wrapping/clipping
    const uint32_t temp = (uint32_t) PWM_PREBUF * duty_cycle_param;
    // Scale back to the correct range
    TCA0.SINGLE.CMP1 = temp >> 8;
}

/***** Tacho *****/

void TCB_enable_event_routing(void) {
    // Configure Event System to route PA6 to TCB0
    EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_PORTA_PIN6_gc; // Pin PA6
    EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc; // route to TCB0
}

void TCB_disable_event_routing(void) {
    // Disable the channel
    EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_OFF_gc;
}

void TCB_init_tacho_counter(void) {
    // Configure PA6 as input
    PORTA.DIRCLR = PIN6_bm; // Set PA6 as input
    PORTA.PIN6CTRL = PORT_ISC_RISING_gc; // Enable sensing on rising edge

    TCB_enable_event_routing();

    // Configure TCB0 in event counting mode
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Enable TCB0 with no prescaler
    TCB0.CTRLB = TCB_CNTMODE_FRQ_gc; // Frequency measurement mode (event counting)

    // Clear the counter
    TCB0.CNT = 0;
}

uint16_t TCB_get_and_reset_count(void) {
    // Disable the event generator to prevent race conditions
    TCB_disable_event_routing();

    // Read the current count
    const uint16_t count = TCB0.CNT;
    // Clear the counter
    TCB0.CNT = 0;

    // Enable event generator again
    TCB_enable_event_routing();

    // Return the current counter value
    return count;
}

void RTC_init(void) {
    // Wait until RTC is not busy
    while (RTC.STATUS > 0)
        /* busy wait */;

    // Initialize RTC function
    // (ATtiny 202 Datasheet 22.3)

    // Configure RTC clock source to use the 1 kHz internal oscillator
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;

    // Set the period to 1000 (1 second with 1 kHz clock)
    RTC.PER = 1000;

    // Enable the overflow (OVF) interrupt
    RTC.INTCTRL = RTC_OVF_bm;

    // Configure the prescaler to divide by 1 and enable the RTC
    RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm;

    // Clear any existing interrupt flags
    RTC.INTFLAGS = RTC_OVF_bm;
}

void RTC_reset(void) {
    // Wait until RTC is not busy
    while (RTC.STATUS > 0)
        /* busy wait */;

    // Reset the RTC counter
    RTC.CNT = 0;
}

ISR(RTC_CNT_vect) {
    // We are reading the TCB0 counter once per second
    const uint16_t revolutions_per_second = TCB_get_and_reset_count();

    // Convert to RPM according to specification
    // See https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf for details.
    // There are two pulses per revolution.
    const uint16_t rpm = revolutions_per_second * 60 >> 1;

    // Atomic store
    cli();
    tacho_rpm = rpm;
    sei();

    // Clear the overflow interrupt flag
    RTC.INTFLAGS = RTC_OVF_bm;
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
    if (data != NULL) {
        // Copy data to the transmission buffer
        tx_length = (length > TWI_BUFFER_SIZE) ? TWI_BUFFER_SIZE : length;
        // This loop is likely to be unrolled by the compiler
        for (uint8_t i = 0; i < tx_length; i++) {
            tx_buffer[i] = data[i];
        }
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
                const uint8_t pwm_target_param = data[0];
                update_target_pwm(pwm_target_param);
                // no response
                TWI_prepare_response(NULL, 0);
            }
            break;
        case TWI_CMD_GET_TACHO:
            // Atomic access
            cli();
            const uint16_t rpm = tacho_rpm;
            sei();
            TWI_prepare_response((const uint8_t *) &rpm, sizeof(rpm));
            break;
        default:
            // no data
            TWI_prepare_response(NULL, 0);
            break;
    }
}

/***** Sleep Mode *****/

void SLEEP_init(void) {
    // Configure PA7 as input
    PORTA.DIRCLR = PIN7_bm;

    // Enable interrupt on both edges with pull-up
    PORTA.PIN7CTRL = PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm;
}

ISR(PORTA_PORT_vect) {
    // ISR(PORTA_PORT_vect) handles interrupts on PA7, which is configured for both edge detection.
    // This ISR is not triggered by PA6, as PA6 is configured for event routing via the Event System
    // and does not generate pin interrupts.

    if (PORTA.INTFLAGS & PIN7_bm) {
        if (!(PORTA.IN & PIN7_bm)) {
            // Falling edge: Enter sleep mode

            // Set PWM to 0%
            update_target_pwm(0);
            // Stop tacho calculations
            TCB_disable_event_routing();
            // Set sleep mode
            set_sleep_mode(SLEEP_MODE_STANDBY);
            sleep_enable();
            // Enter sleep mode
            sleep_cpu();
            // Disable sleep after waking up
            sleep_disable();
        } else {
            // Rising edge: Wake up and restore state

            // Restore PWM duty cycle
            update_target_pwm(pwm_target);

            // Reset tacho calculation
            TCB_get_and_reset_count();
            RTC_reset();

            // Resume tacho calculations
            TCB_enable_event_routing();
        }
        // Clear interrupt flag
        PORTA.INTFLAGS = PIN7_bm;
    }
}

/***** Main *****/

void set_clock_to_20MHz(void) {
    // Set the main clock source to the 20 MHz internal oscillator
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);

    // Set the pre-scaler to divide by 2 (20 MHz / 2 = 10 MHz)
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
}

int main(void) {
    set_clock_to_20MHz();
    TWI_slave_init();
    TCA_init_pwm();
    TCB_init_tacho_counter();
    RTC_init();

    // ReSharper disable once CppDFAEndlessLoop
    while (1);
}
