#include <hardware/structs/io_bank0.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// UART configuration
#define UART_ID uart1
#define BAUD_RATE 9600

// UART pins
#define GPS_TX_PIN 7
#define GPS_RX_PIN 8

// I2C
#define I2C_SDA_PIN_1 4
#define I2C_SCL_PIN_1 5

// I2C for something
#define I2C_SDA_PIN_2 10
#define I2C_SCL_PIN_2 11



void setup_uart() {
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    
    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
}

void read_gps_data() {
    while (uart_is_readable(UART_ID)) {
        char received = uart_getc(UART_ID);
        printf("%c", received);  // Print to USB serial output
    }
}

int main() {
    stdio_init_all();
    setup_uart();

    while (true) {
        read_gps_data();  // Continuously read GPS data
    }

    return 0;
}