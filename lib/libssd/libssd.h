//
// Created by mikaib on 9/10/25.
//

#ifndef LIBSSD_H
#define LIBSSD_H

#include <avr/io.h>
#include <libpin.h>

// max displays per config
#ifndef SSD_MAX_DISPLAYS
#define SSD_MAX_DISPLAYS 4
#endif

// character bitfield
typedef struct ssd_char {
    uint8_t char_t  : 1;
    uint8_t char_tl : 1;
    uint8_t char_tr : 1;
    uint8_t char_m  : 1;
    uint8_t char_bl : 1;
    uint8_t char_br : 1;
    uint8_t char_b  : 1;
} ssd_char;

// characters
#define SSD_CHAR_NONE (ssd_char){ 0, 0, 0, 0, 0, 0, 0 }
#define SSD_CHAR_0    (ssd_char){ 1, 1, 1, 0, 1, 1, 1 }
#define SSD_CHAR_1    (ssd_char){ 0, 0, 1, 0, 0, 1, 0 }
#define SSD_CHAR_2    (ssd_char){ 1, 0, 1, 1, 1, 0, 1 }
#define SSD_CHAR_3    (ssd_char){ 1, 0, 1, 1, 0, 1, 1 }
#define SSD_CHAR_4    (ssd_char){ 0, 1, 1, 1, 0, 1, 0 }
#define SSD_CHAR_5    (ssd_char){ 1, 1, 0, 1, 0, 1, 1 }
#define SSD_CHAR_6    (ssd_char){ 1, 1, 0, 1, 1, 1, 1 }
#define SSD_CHAR_7    (ssd_char){ 1, 0, 1, 0, 0, 1, 0 }
#define SSD_CHAR_8    (ssd_char){ 1, 1, 1, 1, 1, 1, 1 }
#define SSD_CHAR_9    (ssd_char){ 1, 1, 1, 1, 0, 1, 1 }
#define SSD_CHAR_A    (ssd_char){ 1, 1, 1, 1, 1, 1, 0 }
#define SSD_CHAR_B    (ssd_char){ 0, 1, 0, 1, 1, 1, 1 }
#define SSD_CHAR_C    (ssd_char){ 1, 1, 0, 0, 1, 0, 1 }
#define SSD_CHAR_D    (ssd_char){ 0, 0, 1, 1, 1, 1, 1 }
#define SSD_CHAR_E    (ssd_char){ 1, 1, 0, 1, 1, 0, 1 }
#define SSD_CHAR_F    (ssd_char){ 1, 1, 0, 1, 1, 0, 0 }

// conf: display
typedef struct ssd_display {
    // display pins
    pin_def display_top;
    pin_def display_top_left;
    pin_def display_top_right;
    pin_def display_middle;
    pin_def display_bottom_left;
    pin_def display_bottom_right;
    pin_def display_bottom;
    pin_def display_dot;

    // multiplexing
    uint8_t display_count;
    pin_def multiplex[SSD_MAX_DISPLAYS];

    // state
    ssd_char state[SSD_MAX_DISPLAYS];
    uint8_t dot_state[SSD_MAX_DISPLAYS];
} ssd_display;

void ssd_init(ssd_display* disp);
void ssd_render_char(ssd_display disp, uint8_t sub, ssd_char value);
void ssd_render(ssd_display disp);
void ssd_write_char(ssd_display* disp, uint8_t sub, ssd_char value);
void ssd_write_int(ssd_display* disp, uint16_t value);
void ssd_write_hex(ssd_display* disp, uint16_t value);
void ssd_write_dot(ssd_display* disp, uint8_t sub, uint8_t state);
void ssd_shutdown(ssd_display disp);

#endif //LIBSSD_H