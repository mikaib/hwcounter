//
// Created by mikaib on 9/10/25.
//

#include "libpin.h"

void pin_set_mode(const pin_def pin, const uint8_t mode) {
    switch (mode) {
        case PIN_MODE_INPUT:
            *pin.addr_ddr &= ~(1 << pin.offset);
            *pin.addr_port &= ~(1 << pin.offset);
            break;
        case PIN_MODE_INPUT_PULLUP:
            *pin.addr_ddr &= ~(1 << pin.offset);
            *pin.addr_port |= (1 << pin.offset);
            break;
        case PIN_MODE_OUTPUT:
            *pin.addr_ddr |= (1 << pin.offset);
            *pin.addr_port &= ~(1 << pin.offset);
            break;
        default:
            break;
    }
}

void pin_set_state(const pin_def pin, const uint8_t state) {
    switch (state) {
        case PIN_STATE_OFF:
            *pin.addr_port &= ~(1 << pin.offset);
            break;
        case PIN_STATE_ON:
            *pin.addr_port |= (1 << pin.offset);
            break;
        default:
            break;
    }
}

uint8_t pin_get_state(const pin_def pin) {
    return *pin.pin_port & (1 << pin.offset);
}