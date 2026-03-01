/*
 * LED DRIVER — ARDUX XIAO
 *
 * Provides per-key lighting and battery status.
 * 
 * Layout:
 * [0-4]   Battery Status (First 5)
 * [5-12]  Key Backlights (Last 8) -> Mapped to D0-D7
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/battery.h>
#include <zmk/endpoints.h>
#include <string.h>

LOG_MODULE_REGISTER(led_driver, 4);

#define TOTAL_LEDS 13
#define BATT_LEDS  5
#define KEY_LEDS   8

static struct led_rgb pixels[TOTAL_LEDS];
static const struct device *strip;

static int strip_update(void) {
    if (!device_is_ready(strip)) return -ENODEV;
    return led_strip_update_rgb(strip, pixels, TOTAL_LEDS);
}

/* Set Key LEDs (5-12) based on key state */
static void update_key_led(uint8_t index, bool pressed) {
    int led_idx = BATT_LEDS + index;
    if (led_idx >= TOTAL_LEDS) return;

    if (pressed) {
        pixels[led_idx].r = 20; // Dim Yellow when pressed
        pixels[led_idx].g = 20;
        pixels[led_idx].b = 0;
    } else {
        pixels[led_idx].r = 0;  // Dim Cyan when idle
        pixels[led_idx].g = 5;
        pixels[led_idx].b = 5;
    }
}

/* Set Battery LEDs (0-4) based on percentage */
static void update_battery(uint8_t percent) {
    int lit_count = (percent * BATT_LEDS) / 100;
    if (lit_count == 0 && percent > 0) lit_count = 1;

    for (int i = 0; i < BATT_LEDS; i++) {
        if (i < lit_count) {
            pixels[i].r = 0;
            pixels[i].g = 5; // Dim Green
            pixels[i].b = 0;
        } else {
            pixels[i].r = 0;
            pixels[i].g = 0;
            pixels[i].b = 0;
        }
    }
}

static int led_driver_init(void) {
    strip = DEVICE_DT_GET(DT_NODELABEL(led_strip));
    if (!device_is_ready(strip)) return -ENODEV;

    // Initial State: All keys idle
    for (int i = 0; i < KEY_LEDS; i++) {
        update_key_led(i, false);
    }
    update_battery(zmk_battery_state_of_charge());
    strip_update();

    LOG_INF("Ardux LED Driver Active: BATT[0-4], KEYS[5-12]");
    return 0;
}

/* Battery Event Listener */
static int battery_listener(const zmk_event_t *eh) {
    struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    update_battery(ev->state_of_charge);
    strip_update();
    return 0;
}

/* Key Press Event Listener */
static int position_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    update_key_led(ev->position, ev->state);
    strip_update();
    return 0;
}

ZMK_LISTENER(led_driver, battery_listener);
ZMK_SUBSCRIPTION(led_driver, zmk_battery_state_changed);

ZMK_LISTENER(led_driver_keys, position_state_changed_listener);
ZMK_SUBSCRIPTION(led_driver_keys, zmk_position_state_changed);

SYS_INIT(led_driver_init, APPLICATION, 99);
