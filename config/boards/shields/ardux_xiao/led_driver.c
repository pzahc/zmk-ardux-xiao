/*
 * LED driver for ARDUX XIAO
 *
 * Drives a single WS2812B strip with two zones, plus a separate
 * GPIO wake/sleep indicator LED on D9 (P1.14).
 *
 * Zone 1: Battery indicator LEDs (indices 0 to BATTERY_LED_COUNT-1)
 *   - Single color (cyan), fill level = charge level
 *   - Unlit LEDs are OFF
 *   - Red blink when critically low (<10%)
 *   - Gentle pulse when charging on USB
 *
 * Zone 2: Per-key LEDs (indices BATTERY_LED_COUNT to TOTAL_LEDS-1)
 *   - Color reflects the active ARDUX layer
 *   - Brightens on keypress, dims on release
 *
 * Brightness is reduced when running on battery (vs USB).
 * On sleep: strip is blanked, wake LED turns off.
 * On wake: strip resumes, wake LED turns on.
 *
 * Physical LED order on strip (left hand):
 *   LED 0-2:  Battery indicator
 *   LED 3:    A key (position 3)
 *   LED 4:    R key (position 2)
 *   LED 5:    T key (position 1)
 *   LED 6:    S key (position 0)
 *   LED 7:    O key (position 4)
 *   LED 8:    I key (position 5)
 *   LED 9:    Y key (position 6)
 *   LED 10:   E key (position 7)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <hal/nrf_power.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/keymap.h>
#include <zmk/battery.h>
#include <zmk/usb.h>
#include <zmk/activity.h>
#include <string.h>

LOG_MODULE_REGISTER(led_driver, 4);

/* ============================================================
 * Configuration
 * ============================================================ */

#define NUM_KEYS     8
#define NUM_KEY_LEDS 8

/* Number of battery indicator LEDs (first N on the strip).
 * Change this and chain-length in the overlay to match. */
#define BATTERY_LED_COUNT 3

#define TOTAL_LEDS (BATTERY_LED_COUNT + NUM_KEY_LEDS)

/* Battery LED brightness (0-255).
 * These are passed to scale_rgb() which multiplies the base color
 * channel (0-255) by this value / 255.  So brightness 30 with a
 * channel value of 200 gives (200*30)/255 ≈ 24. */
#define BATTERY_BRIGHTNESS        30
#define BATTERY_CHARGE_BRIGHTNESS 40
#define BATTERY_CRIT_BRIGHTNESS   40

/* Blink / pulse intervals (ms) */
#define PULSE_CHARGE_MS   2000
#define BLINK_CRITICAL_MS  400

/* Battery threshold */
#define BATTERY_CRITICAL 10

/* Keypress brightness boost */
#define PRESS_BOOST 90

/* Brightness scale when on battery vs USB (percentage, 0-100).
 * NOTE: Currently the strip VCC is on the 5V pin (USB only),
 * so the strip has no power on battery. This scaling is here
 * for when the strip VCC is rewired to 3V3 or VBAT. */
#define BATTERY_BRIGHTNESS_PCT 50

/* Periodic refresh interval (ms). ~16 FPS — gentle on resources,
 * fast enough for responsive key lighting. */
#define REFRESH_INTERVAL_MS 60

/* ============================================================
 * Key-to-LED mapping
 *
 * Left-hand keymap positions (from ardux_xiao_left.keymap):
 *   pos 0 = S, pos 1 = T, pos 2 = R, pos 3 = A
 *   pos 4 = O, pos 5 = I, pos 6 = Y, pos 7 = E
 *
 * Physical LED wiring (after battery LEDs):
 *   offset 0 = A, 1 = R, 2 = T, 3 = S
 *   offset 4 = O, 5 = I, 6 = Y, 7 = E
 * ============================================================ */

static const uint8_t key_to_led[NUM_KEYS] = {
    BATTERY_LED_COUNT + 3, /* pos 0 (S) -> offset 3 */
    BATTERY_LED_COUNT + 2, /* pos 1 (T) -> offset 2 */
    BATTERY_LED_COUNT + 1, /* pos 2 (R) -> offset 1 */
    BATTERY_LED_COUNT + 0, /* pos 3 (A) -> offset 0 */
    BATTERY_LED_COUNT + 4, /* pos 4 (O) -> offset 4 */
    BATTERY_LED_COUNT + 5, /* pos 5 (I) -> offset 5 */
    BATTERY_LED_COUNT + 6, /* pos 6 (Y) -> offset 6 */
    BATTERY_LED_COUNT + 7, /* pos 7 (E) -> offset 7 */
};

/* ============================================================
 * Layer colors
 *
 * ARDUX layers from ardux.dtsi:
 *   0=Base, 1=Numbers, 2=Symbols, 3=Parentheticals,
 *   4=Navigation, 5=BT Select, 6=Mouse, 7=Custom,
 *   8=Big Sym, 9=Big Function
 * ============================================================ */

#define NUM_LAYER_COLORS 10

static const struct led_rgb layer_colors[NUM_LAYER_COLORS] = {
    [0] = { .r = 10, .g = 30, .b = 35 }, /* Base:           teal */
    [1] = { .r = 15, .g =  8, .b = 45 }, /* Numbers:        deep violet */
    [2] = { .r = 40, .g =  0, .b = 35 }, /* Symbols:        magenta */
    [3] = { .r = 45, .g = 15, .b = 10 }, /* Parentheticals: coral */
    [4] = { .r =  0, .g = 25, .b = 50 }, /* Navigation:     electric blue */
    [5] = { .r = 45, .g =  0, .b = 45 }, /* BT Select:      hot pink */
    [6] = { .r = 40, .g = 30, .b =  0 }, /* Mouse:          amber */
    [7] = { .r = 20, .g = 40, .b =  5 }, /* Custom:         lime */
    [8] = { .r = 25, .g = 10, .b = 40 }, /* Big Sym:        lavender */
    [9] = { .r =  0, .g = 35, .b = 30 }, /* Big Function:   aquamarine */
};

/* ============================================================
 * Battery LED color — single color, fill-level indicates charge
 * ============================================================ */

static const struct led_rgb BAT_COLOR      = { .r =   0, .g = 180, .b = 200 }; /* cyan */
static const struct led_rgb BAT_COLOR_CRIT = { .r = 255, .g =   0, .b =   0 }; /* red */

/* ============================================================
 * State
 * ============================================================ */

static struct led_rgb pixels[TOTAL_LEDS];
static bool key_pressed[NUM_KEYS];
static const struct device *strip;

static uint8_t battery_level = 100;
static bool usb_powered = false;
static bool is_sleeping = false;

/* Wake/sleep indicator LED on D9 */
static const struct gpio_dt_spec wake_led =
    GPIO_DT_SPEC_GET(DT_NODELABEL(wake_led), gpios);

/* Periodic refresh */
static void refresh_work_handler(struct k_work *work);
K_WORK_DEFINE(refresh_work, refresh_work_handler);

static void refresh_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(refresh_timer, refresh_timer_handler, NULL);

/* Blink tick counter */
static uint16_t blink_tick = 0;

/* ============================================================
 * Helpers
 * ============================================================ */

static inline uint8_t clamp_add(uint8_t a, uint8_t b) {
    uint16_t sum = (uint16_t)a + (uint16_t)b;
    return (sum > 255) ? 255 : (uint8_t)sum;
}

static inline struct led_rgb scale_rgb(struct led_rgb c, uint8_t brightness) {
    return (struct led_rgb){
        .r = (uint8_t)(((uint16_t)c.r * brightness) / 255),
        .g = (uint8_t)(((uint16_t)c.g * brightness) / 255),
        .b = (uint8_t)(((uint16_t)c.b * brightness) / 255),
    };
}

/* Apply global brightness reduction when on battery */
static inline struct led_rgb apply_power_scale(struct led_rgb c) {
    if (usb_powered) {
        return c;
    }
    return (struct led_rgb){
        .r = (uint8_t)(((uint16_t)c.r * BATTERY_BRIGHTNESS_PCT) / 100),
        .g = (uint8_t)(((uint16_t)c.g * BATTERY_BRIGHTNESS_PCT) / 100),
        .b = (uint8_t)(((uint16_t)c.b * BATTERY_BRIGHTNESS_PCT) / 100),
    };
}

/* ============================================================
 * Refresh work handler
 * ============================================================ */

static void refresh_work_handler(struct k_work *work) {
    if (!device_is_ready(strip)) {
        return;
    }

    /* If sleeping, blank the strip and stop */
    if (is_sleeping) {
        memset(pixels, 0, sizeof(pixels));
        led_strip_update_rgb(strip, pixels, TOTAL_LEDS);
        return;
    }

    /* --- Poll USB and battery state --- */

    /* Read VBUS detect directly from nRF52840 POWER register —
     * ZMK's zmk_usb_get_status() was returning USB_DC_UNKNOWN
     * even when USB is clearly connected (CDC ACM works). */
    usb_powered = nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
    battery_level = zmk_battery_state_of_charge();

    /* --- Tick counter for blink/pulse --- */

    blink_tick++;

    /* --- Zone 1: Battery LEDs ---
     *
     * Simple scheme:
     *   - One color (cyan), fill level = number of lit LEDs
     *   - Charging on USB: gentle pulse (brightness ramps up/down)
     *   - Critical (<10%): red blink
     *   - Otherwise: steady cyan, lit LEDs = charge level
     *   - Unlit LEDs are OFF (no ghost)
     */

    int leds_active;
    if (BATTERY_LED_COUNT == 1) {
        leds_active = 1;
    } else {
        leds_active = (battery_level * BATTERY_LED_COUNT + 99) / 100;
        if (leds_active > BATTERY_LED_COUNT) {
            leds_active = BATTERY_LED_COUNT;
        }
    }

    bool is_critical = (battery_level <= BATTERY_CRITICAL);

    for (int i = 0; i < BATTERY_LED_COUNT; i++) {
        if (usb_powered) {
            /* On USB: always show red pulse, all active LEDs */
            if (i < leds_active) {
                uint16_t pulse_ticks = PULSE_CHARGE_MS / REFRESH_INTERVAL_MS;
                uint16_t pos = blink_tick % pulse_ticks;
                uint16_t half = pulse_ticks / 2;
                uint8_t bright = (pos < half)
                    ? (uint8_t)(BATTERY_CHARGE_BRIGHTNESS * pos / half)
                    : (uint8_t)(BATTERY_CHARGE_BRIGHTNESS * (pulse_ticks - pos) / half);
                if (bright < 10) bright = 10;
                pixels[i] = scale_rgb(BAT_COLOR_CRIT, bright);
            } else {
                pixels[i] = (struct led_rgb){0, 0, 0};
            }
        } else if (is_critical) {
            /* On battery, critical (<10%): red blink all LEDs */
            uint16_t crit_ticks = BLINK_CRITICAL_MS / REFRESH_INTERVAL_MS;
            bool blink_on = (blink_tick % (crit_ticks * 2)) < crit_ticks;
            pixels[i] = blink_on
                ? scale_rgb(BAT_COLOR_CRIT, BATTERY_CRIT_BRIGHTNESS)
                : (struct led_rgb){0, 0, 0};
        } else if (battery_level < 50) {
            /* On battery, low (10-49%): steady red fill */
            if (i < leds_active) {
                pixels[i] = scale_rgb(BAT_COLOR_CRIT, BATTERY_BRIGHTNESS);
            } else {
                pixels[i] = (struct led_rgb){0, 0, 0};
            }
        } else {
            /* On battery, 50%+: off */
            pixels[i] = (struct led_rgb){0, 0, 0};
        }
    }

    /* --- Zone 2: Per-key LEDs --- */

    uint8_t layer = zmk_keymap_highest_layer_active();
    if (layer >= NUM_LAYER_COLORS) {
        layer = 0;
    }

    struct led_rgb base = layer_colors[layer];

    for (int i = 0; i < NUM_KEYS; i++) {
        uint8_t led_idx = key_to_led[i];
        struct led_rgb c;
        if (key_pressed[i]) {
            c.r = clamp_add(base.r, PRESS_BOOST);
            c.g = clamp_add(base.g, PRESS_BOOST);
            c.b = clamp_add(base.b, PRESS_BOOST);
        } else {
            c = base;
        }
        pixels[led_idx] = apply_power_scale(c);
    }

    /* Push to hardware */
    int rc = led_strip_update_rgb(strip, pixels, TOTAL_LEDS);
    if (rc) {
        LOG_ERR("strip update failed: %d", rc);
    }
}

/* Timer fires periodically, submits work to the system work queue */
static void refresh_timer_handler(struct k_timer *timer) {
    k_work_submit(&refresh_work);
}

/* ============================================================
 * ZMK Event Handlers
 * ============================================================ */

static int on_position_state_changed(const zmk_event_t *eh) {
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev && ev->position < NUM_KEYS) {
        key_pressed[ev->position] = ev->state;
    }
    return 0;
}

static int on_layer_state_changed(const zmk_event_t *eh) {
    return 0;
}

/* Battery and USB state are polled in refresh_work_handler —
 * event-driven updates were unreliable. */

static int on_activity_state_changed(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev) {
        bool was_sleeping = is_sleeping;
        is_sleeping = (ev->state == ZMK_ACTIVITY_SLEEP);

        /* Update wake LED */
        if (device_is_ready(wake_led.port)) {
            gpio_pin_set_dt(&wake_led, !is_sleeping);
        }

        if (is_sleeping && !was_sleeping) {
            /* Entering sleep: stop the timer first, then blank the strip
             * SYNCHRONOUSLY.  k_work_submit() is too late — the MCU
             * enters deep sleep before the system work queue runs. */
            k_timer_stop(&refresh_timer);
            memset(pixels, 0, sizeof(pixels));
            led_strip_update_rgb(strip, pixels, TOTAL_LEDS);
            LOG_INF("LED driver: sleep (strip blanked sync)");
        } else if (!is_sleeping && was_sleeping) {
            /* Waking up: restart the periodic refresh */
            k_timer_start(&refresh_timer,
                          K_MSEC(REFRESH_INTERVAL_MS),
                          K_MSEC(REFRESH_INTERVAL_MS));
            LOG_INF("LED driver: wake");
        }
    }
    return 0;
}

/* ============================================================
 * Event subscriptions
 * ============================================================ */

ZMK_LISTENER(led_driver, on_position_state_changed);
ZMK_SUBSCRIPTION(led_driver, zmk_position_state_changed);

ZMK_LISTENER(led_driver_layer, on_layer_state_changed);
ZMK_SUBSCRIPTION(led_driver_layer, zmk_layer_state_changed);

/* Battery and USB listeners removed — state is polled in refresh handler */

ZMK_LISTENER(led_driver_activity, on_activity_state_changed);
ZMK_SUBSCRIPTION(led_driver_activity, zmk_activity_state_changed);

/* ============================================================
 * Initialization
 * ============================================================ */

static int led_driver_init(void) {
    strip = DEVICE_DT_GET(DT_NODELABEL(led_strip));
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    /* Initialize wake/sleep LED — ON at boot (board is awake) */
    if (device_is_ready(wake_led.port)) {
        gpio_pin_configure_dt(&wake_led, GPIO_OUTPUT_ACTIVE);
        LOG_INF("Wake LED initialized on D9");
    } else {
        LOG_WRN("Wake LED GPIO not ready");
    }

    battery_level = zmk_battery_state_of_charge();
    usb_powered = zmk_usb_is_powered();

    /* Initial refresh (no contention at boot) */
    refresh_work_handler(NULL);

    /* Start periodic refresh timer */
    k_timer_start(&refresh_timer,
                  K_MSEC(REFRESH_INTERVAL_MS),
                  K_MSEC(REFRESH_INTERVAL_MS));

    LOG_INF("LED driver active: %d bat + %d key = %d LEDs, bat=%d%% usb=%d, refresh=%dms",
            BATTERY_LED_COUNT, NUM_KEY_LEDS, TOTAL_LEDS,
            battery_level, usb_powered, REFRESH_INTERVAL_MS);
    return 0;
}

SYS_INIT(led_driver_init, APPLICATION, 99);
