#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"


/*
* Possible UART Interface Configurations for u-blox M8:
| Baud Rate | Data Bits | Parity | Stop Bits |
|-----------|-----------|--------|-----------|
| 4800      | 8         | None   | 1         |
| 9600      | 8         | None   | 1         |
| 19200     | 8         | None   | 1         |
| 38400     | 8         | None   | 1         |
| 57600     | 8         | None   | 1         |
| 115200    | 8         | None   | 1         |
| 230400    | 8         | None   | 1         |
| 460800    | 8         | None   | 1         |
*/

#define BAUD_RATE 230400

// UART configuration
#define UART_ID uart1

// All Numbers are the GPIO Number not the Pin Number
// I2C for IMU
#define I2C_SDA_PIN_1 4
#define I2C_SCL_PIN_1 5

// UART pins for GPS (BN880)
#define GPS_TX_PIN 8
#define GPS_RX_PIN 9

// I2C for Magnetometer (BN880)
#define I2C_SDA_PIN_2 10
#define I2C_SCL_PIN_2 11

void setup_uart() {
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    
    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
}

void relay_usb_to_uart() {
    while (true) {
        if (stdio_usb_connected()) {
            // Read from USB serial and send to GPS UART
            while (uart_is_writable(UART_ID) && getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
                uart_putc(UART_ID, getchar());
            }

            // Read from GPS UART and send to USB serial
            while (uart_is_readable(UART_ID)) {
                putchar(uart_getc(UART_ID));
            }
        }
    }
}

int main() {
    stdio_init_all();
    setup_uart();

    relay_usb_to_uart();

    return 0;
}