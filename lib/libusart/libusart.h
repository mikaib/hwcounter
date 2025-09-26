#ifndef LIBUSART_H
#define LIBUSART_H

#include <avr/io.h>

// conf: parity bit
#define USART_PARITY_OFF  (0<<UPM00)
#define USART_PARITY_EVEN (2<<UPM00)
#define USART_PARITY_ODD  (3<<UPM00)

// conf: stop bit
#define USART_STOP_ONE    (0<<USBS0)
#define USART_STOP_TWO    (1<<USBS0)

// conf: data size
#define USART_BIT_FIVE    (0<<UCSZ00)
#define USART_BIT_SIX     (1<<UCSZ00)
#define USART_BIT_SEVEN   (2<<UCSZ00)
#define USART_BIT_EIGHT   (3<<UCSZ00)

// conf: async
#define USART_ASYNC       (0<<UMSEL00)

// conf: cpu clock
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// conf: usart
typedef struct usart_config {
    uint32_t baud; // 9600
    uint8_t flags; // USART_PARITY_OFF | USART_ASYNC | USART_STOP_ONE | USART_BIT_EIGHT
} usart_config;

void usart_init(usart_config config);
void usart_await_ready();
void usart_write_byte(char data);
void usart_write_str(const char* data);
void usart_shutdown();

#endif //LIBUSART_H