//
// Created by mikaib on 9/10/25.
//

#include <util/delay.h>
#include "libssd.h"

ssd_char g_ssd_conv[16] = {
    SSD_CHAR_0,
    SSD_CHAR_1,
    SSD_CHAR_2,
    SSD_CHAR_3,
    SSD_CHAR_4,
    SSD_CHAR_5,
    SSD_CHAR_6,
    SSD_CHAR_7,
    SSD_CHAR_8,
    SSD_CHAR_9,
    SSD_CHAR_A,
    SSD_CHAR_B,
    SSD_CHAR_C,
    SSD_CHAR_D,
    SSD_CHAR_E,
    SSD_CHAR_F,
};

void ssd_init(ssd_display* disp) {
    for (uint8_t idx = 0; idx < disp->display_count; idx++) {
        pin_set_mode(disp->multiplex[idx], PIN_MODE_OUTPUT);
        pin_set_state(disp->multiplex[idx], PIN_STATE_ON);
        disp->state[idx] = SSD_CHAR_NONE;
        disp->dot_state[idx] = 0;
    }

    pin_set_mode(disp->display_top, PIN_MODE_OUTPUT);
    pin_set_mode(disp->display_top_left, PIN_MODE_OUTPUT);
    pin_set_mode(disp->display_top_right, PIN_MODE_OUTPUT);
    pin_set_mode(disp->display_middle, PIN_MODE_OUTPUT);
    pin_set_mode(disp->display_bottom_left, PIN_MODE_OUTPUT);
    pin_set_mode(disp->display_bottom_right, PIN_MODE_OUTPUT);
    pin_set_mode(disp->display_bottom, PIN_MODE_OUTPUT);
    pin_set_mode(disp->display_dot, PIN_MODE_OUTPUT);
}

void ssd_render_char(const ssd_display disp, const uint8_t sub, const ssd_char value) {
    for (uint8_t idx = 0; idx < disp.display_count; idx++) {
        pin_set_state(disp.multiplex[idx], PIN_STATE_ON);
    }

    pin_set_state(disp.display_top, value.char_t);
    pin_set_state(disp.display_top_left, value.char_tl);
    pin_set_state(disp.display_top_right, value.char_tr);
    pin_set_state(disp.display_middle, value.char_m);
    pin_set_state(disp.display_bottom_left, value.char_bl);
    pin_set_state(disp.display_bottom_right, value.char_br);
    pin_set_state(disp.display_bottom, value.char_b);
    pin_set_state(disp.display_dot, disp.dot_state[sub]);

    pin_set_state(disp.multiplex[sub], PIN_STATE_OFF);
}

void ssd_render(const ssd_display disp) {
    for (uint8_t idx = 0; idx < disp.display_count; idx++) {
        ssd_render_char(disp, idx, disp.state[idx]);
        _delay_ms(5);
    }
}

void ssd_write_char(ssd_display* disp, const uint8_t sub, const ssd_char value) {
    if (sub > SSD_MAX_DISPLAYS || sub < 0) {
        return;
    }

    disp->state[sub] = value;
}

void ssd_write_int(ssd_display* disp, const uint16_t value) {
    int pos = disp->display_count - 1;
    int div = 1;

    for (uint8_t idx = 0; idx < disp->display_count; idx++) {
        int digit = (value / div) % 10;
        ssd_write_char(disp, pos, g_ssd_conv[digit]);

        div *= 10;
        pos--;
    }
}

void ssd_write_hex(ssd_display* disp, const uint16_t value) {
    int pos = disp->display_count - 1;
    uint16_t temp = value;

    for (uint8_t idx = 0; idx < disp->display_count; idx++) {
        int digit = temp & 0x0F;
        ssd_write_char(disp, pos, g_ssd_conv[digit]);

        temp >>= 4;
        pos--;
    }
}

void ssd_write_dot(ssd_display* disp, const uint8_t sub, const uint8_t state) {
    if (sub > SSD_MAX_DISPLAYS || sub < 0) {
        return;
    }

    disp->dot_state[sub] = state;
}

void ssd_shutdown(const ssd_display disp) {
    pin_set_mode(disp.display_top, PIN_MODE_INPUT);
    pin_set_mode(disp.display_top_left, PIN_MODE_INPUT);
    pin_set_mode(disp.display_top_right, PIN_MODE_INPUT);
    pin_set_mode(disp.display_middle, PIN_MODE_INPUT);
    pin_set_mode(disp.display_bottom_left, PIN_MODE_INPUT);
    pin_set_mode(disp.display_bottom_right, PIN_MODE_INPUT);
    pin_set_mode(disp.display_bottom, PIN_MODE_INPUT);
}