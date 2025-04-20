# PWM Fan Controller with Internal USB connection - Firmware

> Firmware for the PWM Fan Controller

Please refer to the [project README](../README.md) for further information.

## API

The MCU pin assignment is as follows:

* `PA1` - SDA
* `PA2` - SCL
* `PA3` - PWM output
* `PA6` - Tachometer signal input
* `PA7` - `SLEEP#` input, active low

The firmware exposes an I2C API for the PWM Fan Controller with the following commands:

* `TWI_CMD_GET_ID` (read): Get the serial number of the ATtiny202 device. Returns the serial number as a 4-byte array.
* `TWI_CMD_SET_PWM` (write): Set the PWM value using an 1-byte parameter, where `0x00` is 0%, aka off, and `0xFF` is
  100%, aka full speed.
* `TWI_CMD_GET_TACHO` (read): Get the most recent tachometer measurement in RPM as a 2-byte array.

Pin `PA7` acts as a suspend pin. If the pin is pulled low, the firmware will enter a low-power mode. The firmware will
wake up when the pin is pulled high again.

## Architecture

The firmware is designed to run on
the [ATtiny202](https://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny202-402-AVR-MCU-with-Core-Independent-Peripherals_and-picoPower-40001969A.pdf)
microcontroller. It uses the I2C protocol for communication with
an [FTDI201XS](https://www.mouser.de/datasheet/2/163/DS_FT201X-3173.pdf) host.

The code makes heavy use of interrupts and the event system to handle all required tasks on a small MCU, so that the
main loop solely consists of setup code.

### TWI handling

TWI communication is handled using the *ATtiny202 TWI*
peripheral with small RX and TX buffers for multibyte transfers. Depending on the command received, a matching handler
function is called.

### PWM generation

The PWM signal is generated using
the *ATtiny202 Timer/Counter TCA*.
The frequency is set to 25 kHz, according to the specifications for PC fans (c.f. [Noctua PWM specifications
white paper](https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf)).

### Tachometer measurement

The tachometer pulses are counted using the *ATtiny202 Timer/Counter TCB*.
Using the *Event System*, the TCB is configured to trigger on the rising edge of the tachometer signal on pin `PA6`.

The *ATtiny202 RTC* is configured to generate an interrupt once per second. The interrupt handler reads the TCB
counter value and resets it. The counter value is then converted to RPM and stored in a buffer, which can be read by the
TWI handler.

### Low-power mode handling

When the `SLEEP#` pin is pulled low, the firmware enters a low-power mode. The PWM signal is turned off and tachometer
measurements is disabled.

The firmware will wake up when the pin is pulled high again, re-setting the PWM duty cycle to the last set value and
re-enabling the tachometer measurement. Please note that the first two seconds of RMP measurement after waking up are
not accurate, as the FAN needs to spin up first and the measurement has been reset to zero.

## License

The firmware is licensed under the [MIT License](LICENSE.txt). See the [LICENSE.txt](LICENSE.txt) file for details.

© 2025 Stefan Haun and contributors
