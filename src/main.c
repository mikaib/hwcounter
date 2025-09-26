#include <libusart.h>
#include <libpin.h>
#include <libssd.h>
#include <string.h>
#include <util/atomic.h>

#define SECONDS_TO_MILLIS 1000
#define MS_TO_KMH 3.6

#define DEFAULT_SENSOR_DIST_M 0.6
#define SPEED_MEASUREMENT_MIN_TIME_MS 0
#define SPEED_MEASUREMENT_MAX_TIME_MS 1000
#define SENSOR_DEBOUNCE_MS 50

ssd_display g_display;
uint64_t g_last_passed        = 0;
uint64_t g_timer              = 0;
uint64_t g_debounce_a         = 0;
uint64_t g_debounce_b         = 0;
uint16_t g_counter            = 0;
float g_speed                 = 0;
float g_dist                  = DEFAULT_SENSOR_DIST_M;
uint16_t g_measurement_min    = SPEED_MEASUREMENT_MIN_TIME_MS;
uint16_t g_measurement_max    = SPEED_MEASUREMENT_MAX_TIME_MS;
uint16_t g_measurement_active = 0;
uint8_t  g_last_state_a       = 0;
uint8_t  g_last_state_b       = 0;

void initialize_io() {
    g_display = (ssd_display){
        .display_top = PIN_D2,
        .display_top_left = PIN_D3,
        .display_top_right = PIN_D4,
        .display_middle = PIN_D5,
        .display_bottom_left = PIN_D6,
        .display_bottom_right = PIN_D7,
        .display_bottom = PIN_D8,
        .display_dot = PIN_D12,
        .display_count = 3,
        .multiplex = { PIN_D10, PIN_D9, PIN_D11 }
    };

    // binary counter
    pin_set_mode(PIN_A0, PIN_MODE_OUTPUT);
    pin_set_mode(PIN_A1, PIN_MODE_OUTPUT);
    pin_set_mode(PIN_A2, PIN_MODE_OUTPUT);
    pin_set_mode(PIN_A3, PIN_MODE_OUTPUT);

    // status led
    pin_set_mode(PIN_D13, PIN_MODE_OUTPUT);

    // sensors
    pin_set_mode(PIN_A4, PIN_MODE_INPUT_PULLUP);
    pin_set_mode(PIN_A5, PIN_MODE_INPUT_PULLUP);

    // 7-segment display
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

uint8_t button_state_a() {
    uint8_t current_state = !pin_get_state(PIN_A4);
    uint64_t now = millis();

    if (now - g_debounce_a >= 0.0 && g_last_state_a != current_state) {
        g_debounce_a = now + SENSOR_DEBOUNCE_MS;
        g_last_state_a = current_state;
        return current_state;
    }

    return 0;
}

uint8_t button_state_b() {
    uint8_t current_state = !pin_get_state(PIN_A5);
    uint64_t now = millis();

    if (now - g_debounce_b >= 0.0 && g_last_state_b != current_state) {
        g_debounce_b = now + SENSOR_DEBOUNCE_MS;
        g_last_state_b = current_state;
        return current_state;
    }

    return 0;
}

uint8_t axel_detected() {
    // the system doesn't really "care" about which sensor is triggered, the measurement will work either way.
    // the main important part is that there *are* multiple sensors. by handling it this way we can avoid redundant logic.
    // note that we can safely do this because the system isn't designed to handle directional traffic anyway (e.g. both increase and decrease the counter based on direction)
    return button_state_a() || button_state_b();
}

void set_status_led(uint8_t state) {
    pin_set_state(PIN_D13, state);
}

void vehicle_passed() {
    uint64_t now = millis();
    uint64_t dt = now - g_last_passed;

    if (axel_detected()) {
        if (dt >= g_measurement_min && dt <= g_measurement_max && g_measurement_active == 1) {
            g_speed = (g_dist / ((float)dt / (float)SECONDS_TO_MILLIS)) * MS_TO_KMH;
            g_counter = (g_counter + 1) % 16;
            g_measurement_active = 0;
            set_status_led(0); // ok
            return;
        }

        if (g_measurement_active == 1) { // too fast or too slow, measurement failed!
            set_status_led(1);
            g_measurement_active = 0;
            return;
        }

        g_last_passed = now;
        g_measurement_active = 1;
    }
}

void poll_measurement_state() {
    uint64_t now = millis();
    uint64_t dt = now - g_last_passed;

    if (g_measurement_active == 1 && (dt > g_measurement_max)) {
        g_measurement_active = 0;
        set_status_led(1); // error
    }
}

void display_counter() {
    // counter
    pin_set_state(PIN_A0, (g_counter >> 0) & 1);
    pin_set_state(PIN_A1, (g_counter >> 1) & 1);
    pin_set_state(PIN_A2, (g_counter >> 2) & 1);
    pin_set_state(PIN_A3, (g_counter >> 3) & 1);

    // write speedometer
    ssd_write_int(&g_display, (uint16_t)(g_speed * 100.0));

    // render speedometer
    if (g_measurement_active == 0) {
        ssd_write_dot(&g_display, 0, 1);
        ssd_render(g_display);
    } else {
        ssd_write_dot(&g_display, 0, 0);
        ssd_render_char(g_display, 0, SSD_CHAR_NONE);
    }
}

int main() {
    init();
    initialize_io();

    for (;;) {
        poll_measurement_state();
        vehicle_passed();
        display_counter();
    }

    return 0;
}

ISR(TIMER1_COMPA_vect) {
    g_timer++;
}