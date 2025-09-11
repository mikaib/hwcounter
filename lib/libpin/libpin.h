//
// Created by mikaib on 9/10/25.
//

#ifndef LIBPIN_H
#define LIBPIN_H

#include <avr/io.h>

typedef struct pin_def {
    volatile uint8_t* addr_ddr;
    volatile uint8_t* addr_port;
    volatile uint8_t* pin_port;
    uint8_t  offset;
} pin_def;

// states
#define PIN_STATE_OFF 0
#define PIN_STATE_ON 1

// modes
#define PIN_MODE_INPUT 0
#define PIN_MODE_INPUT_PULLUP 1
#define PIN_MODE_OUTPUT 2

// res
#define PIN_RESULT_ERROR 0
#define PIN_RESULT_OK 1

// pin mapping for arduino uno
#define PIN_D0  (pin_def){ &DDRD, &PORTD, &PIND, PORTD0 }
#define PIN_D1  (pin_def){ &DDRD, &PORTD, &PIND, PORTD1 }
#define PIN_D2  (pin_def){ &DDRD, &PORTD, &PIND, PORTD2 }
#define PIN_D3  (pin_def){ &DDRD, &PORTD, &PIND, PORTD3 }
#define PIN_D4  (pin_def){ &DDRD, &PORTD, &PIND, PORTD4 }
#define PIN_D5  (pin_def){ &DDRD, &PORTD, &PIND, PORTD5 }
#define PIN_D6  (pin_def){ &DDRD, &PORTD, &PIND, PORTD6 }
#define PIN_D7  (pin_def){ &DDRD, &PORTD, &PIND, PORTD7 }
#define PIN_D8  (pin_def){ &DDRB, &PORTB, &PINB, PORTB0 }
#define PIN_D9  (pin_def){ &DDRB, &PORTB, &PINB, PORTB1 }
#define PIN_D10 (pin_def){ &DDRB, &PORTB, &PINB, PORTB2 }
#define PIN_D11 (pin_def){ &DDRB, &PORTB, &PINB, PORTB3 }
#define PIN_D12 (pin_def){ &DDRB, &PORTB, &PINB, PORTB4 }
#define PIN_D13 (pin_def){ &DDRB, &PORTB, &PINB, PORTB5 }
#define PIN_A0  (pin_def){ &DDRC, &PORTC, &PINC, PORTC0 }
#define PIN_A1  (pin_def){ &DDRC, &PORTC, &PINC, PORTC1 }
#define PIN_A2  (pin_def){ &DDRC, &PORTC, &PINC, PORTC2 }
#define PIN_A3  (pin_def){ &DDRC, &PORTC, &PINC, PORTC3 }
#define PIN_A4  (pin_def){ &DDRC, &PORTC, &PINC, PORTC4 }
#define PIN_A5  (pin_def){ &DDRC, &PORTC, &PINC, PORTC5 }

void pin_set_mode(pin_def pin, uint8_t mode);
void pin_set_state(pin_def pin, uint8_t state);
uint8_t pin_get_state(pin_def pin);

#endif // LIBPIN_H