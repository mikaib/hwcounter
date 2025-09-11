//
// Created by mikaib on 9/9/25.
//

#include "libusart.h"

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void usart_init(usart_config config) {
    // UBRR0 = (cpu_f / (16 * BAUD)) - 1
    unsigned long p = (F_CPU / (16UL * config.baud)) - 1;

    // set registers
    UCSR0C = config.flags;
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    UBRR0 = p;

    // global interrupts
    sei();
}

void usart_await_ready() {
    while ((UCSR0A & (1<<UDRE0)) == 0) {
        asm volatile ("nop");
    };
}

void usart_write_byte(const char data) {
    UDR0 = data;
    usart_await_ready();
}

void usart_write_str(const char* data) {
    const size_t len = strlen(data);
    for (size_t i = 0; i < len; i++) {
        usart_write_byte(data[i]);
    }
}

void usart_shutdown() {
    // disable RX/TX
    UCSR0B &= ~((1<<RXEN0) | (1<<TXEN0));
}