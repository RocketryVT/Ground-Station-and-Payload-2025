#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <string.h>

#define UART_ID uart1
#define UART_BAUD_RATE 9600
#define GPS_TX_PIN 4
#define GPS_RX_PIN 5

void setup_uart() {
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
}

void read_gps_data() {
    while (uart_is_readable(UART_ID)) {
        char received = uart_getc(UART_ID);
        // Process received character
        // You can store it in a buffer or print it
        printf(received);  // Print to standard output (e.g., serial terminal)
    }
}

int main() {
    stdio_init_all();
    setup_uart();

    // Optionally, send a command to the GPS module (if needed)
    // For example, send_command("some_command");

    while (true) {
        read_gps_data();  // Continuously read GPS data
    }

    return 0;
}