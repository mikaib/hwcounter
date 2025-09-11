#include <libusart.h>
#include <libpin.h>
#include <libssd.h>
#include <util/atomic.h>

ssd_display g_display;
uint64_t g_last_passed = 0;
uint16_t g_counter     = 0;
uint64_t g_timer       = 0;
float g_speed          = 0;
float g_dist           = 0.6;

void InitializeIO() {
    g_display = (ssd_display){
        .display_top = PIN_D2,
        .display_top_left = PIN_D3,
        .display_top_right = PIN_D4,
        .display_middle = PIN_D5,
        .display_bottom_left = PIN_D6,
        .display_bottom_right = PIN_D7,
        .display_bottom = PIN_D8,
        .display_count = 4,
        .multiplex = { PIN_D10, PIN_D9, PIN_D11, PIN_D12 }
    };

    pin_set_mode(PIN_A0, PIN_MODE_OUTPUT);
    pin_set_mode(PIN_A1, PIN_MODE_OUTPUT);
    pin_set_mode(PIN_A2, PIN_MODE_OUTPUT);
    pin_set_mode(PIN_A3, PIN_MODE_OUTPUT);
    pin_set_mode(PIN_A4, PIN_MODE_INPUT_PULLUP);
    pin_set_mode(PIN_A5, PIN_MODE_INPUT_PULLUP);

    ssd_init(&g_display);
}

void init() {
    uint64_t overflow = ((F_CPU / 1000) / 8);

    TCCR1B |= (1 << WGM12) | (1 << CS11);
    OCR1AH = (overflow >> 8);
    OCR1AL = overflow;
    TIMSK1 |= (1 << OCIE1A);

    sei();
}

uint64_t millis() {
    uint64_t value = 0;

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        value = g_timer;
    }

    return value;
}

uint8_t button_state() {
    return !pin_get_state(PIN_A4) || !pin_get_state(PIN_A5);
}


uint8_t axel_detected() {
    return button_state();
}

void vehicle_passed() {
    uint64_t now = millis();
    uint64_t dt = now - g_last_passed;

    if (axel_detected()) {
        if (dt >= 1000 && dt <= 2000) {
            g_speed = g_dist / ((float)dt / 1000.0);
            g_counter = (g_counter + 1) % 16;
            g_last_passed += 10000; // disable
            return;
        }

        g_last_passed = now;
    }
}

void display_counter() {
    // counter
    pin_set_state(PIN_A0, (g_counter >> 0) & 1);
    pin_set_state(PIN_A1, (g_counter >> 1) & 1);
    pin_set_state(PIN_A2, (g_counter >> 2) & 1);
    pin_set_state(PIN_A3, (g_counter >> 3) & 1);

    // speedometer
    ssd_write_int(&g_display, (uint16_t)(g_speed * 100.0));
    ssd_render(g_display);
}

int main() {
    init();
    InitializeIO();

    for (;;) {
        vehicle_passed();
        display_counter();
    }

    return 0;
}

ISR(TIMER1_COMPA_vect) {
    g_timer++;
}