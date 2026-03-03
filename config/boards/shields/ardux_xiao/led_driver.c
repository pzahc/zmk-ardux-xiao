/*
 * LED driver for ARDUX XIAO
 *
 * Drives a single WS2812B strip with two zones:
 *
 * Zone 1: Battery indicator LEDs (indices 0 to BATTERY_LED_COUNT-1)
 *   - Gradient from magenta (empty) to cyan (full)
 *   - Proportional fill: active LEDs are bright, inactive are very dim
 *   - Red-only blink when critically low (<10%)
 *   - Slow pulse when charging (cool white)
 *
 * Zone 2: Per-key LEDs (indices BATTERY_LED_COUNT to TOTAL_LEDS-1)
 *   - Color reflects the active ARDUX layer
 *   - Brightens on keypress, dims on release
 *
 * All strip updates are funneled through a single work queue item
 * to prevent race conditions from ARDUX's rapid combo events.
 *
 * Physical LED order on strip (left hand):
 *   LED 0-4:  Battery indicator
 *   LED 5:    A key (position 3)
 *   LED 6:    R key (position 2)
 *   LED 7:    T key (position 1)
 *   LED 8:    S key (position 0)
 *   LED 9:    O key (position 4)
 *   LED 10:   I key (position 5)
 *   LED 11:   Y key (position 6)
 *   LED 12:   E key (position 7)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/keymap.h>
#include <zmk/battery.h>
#include <zmk/usb.h>
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

/* Battery LED brightness (0-255) */
#define BATTERY_ACTIVE_BRIGHTNESS  35
#define BATTERY_INACTIVE_BRIGHTNESS 4
#define BATTERY_BLINK_BRIGHTNESS   60

/* Blink intervals (ms) */
#define BLINK_CHARGE_MS   1500
#define BLINK_CRITICAL_MS  400

/* Battery threshold */
#define BATTERY_CRITICAL 10

/* Keypress brightness boost */
#define PRESS_BOOST 90

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
 * Battery gradient colors
 * ============================================================ */

static const struct led_rgb BAT_COLOR_EMPTY  = { .r = 255, .g =   0, .b = 200 };
static const struct led_rgb BAT_COLOR_FULL   = { .r =   0, .g = 255, .b = 220 };
static const struct led_rgb BAT_COLOR_CRIT   = { .r = 255, .g =   0, .b =   0 };
static const struct led_rgb BAT_COLOR_CHARGE = { .r = 140, .g = 220, .b = 255 };

/* ============================================================
 * State
 *
 * All state is updated atomically from event handlers.
 * The actual pixel computation and SPI transfer happens
 * only in refresh_work_handler on the system work queue,
 * which is single-threaded — no races possible.
 * ============================================================ */

static struct led_rgb pixels[TOTAL_LEDS];
static bool key_pressed[NUM_KEYS];
static const struct device *strip;

static uint8_t battery_level = 100;
static bool usb_powered = false;
static bool blink_on = true;

/* Periodic refresh timer.
 * Instead of refreshing on every event (which causes flickering
 * because ARDUX combos rapidly activate/deactivate layers),
 * we refresh at a fixed rate. By the time the timer fires,
 * combo processing has settled and layer state is stable. */
static void refresh_work_handler(struct k_work *work);
K_WORK_DEFINE(refresh_work, refresh_work_handler);

static void refresh_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(refresh_timer, refresh_timer_handler, NULL);

/* How often to refresh the strip (ms). 33ms ≈ 30 FPS. */
#define REFRESH_INTERVAL_MS 33

/* Blink state counter — toggled by refresh cycle, not a separate timer.
 * Counts refresh ticks to derive blink timing. */
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

static inline struct led_rgb lerp_rgb(struct led_rgb a, struct led_rgb b, uint8_t t) {
    return (struct led_rgb){
        .r = (uint8_t)(((uint16_t)a.r * (255 - t) + (uint16_t)b.r * t) / 255),
        .g = (uint8_t)(((uint16_t)a.g * (255 - t) + (uint16_t)b.g * t) / 255),
        .b = (uint8_t)(((uint16_t)a.b * (255 - t) + (uint16_t)b.b * t) / 255),
    };
}

static struct led_rgb battery_gradient_color(int led_index) {
    if (BATTERY_LED_COUNT <= 1) {
        return lerp_rgb(BAT_COLOR_EMPTY, BAT_COLOR_FULL,
                        (uint8_t)((battery_level * 255) / 100));
    }
    uint8_t t = (uint8_t)(((uint16_t)led_index * 255) / (BATTERY_LED_COUNT - 1));
    return lerp_rgb(BAT_COLOR_EMPTY, BAT_COLOR_FULL, t);
}

/* ============================================================
 * Refresh work handler — ONLY place pixels are computed
 * and SPI transfer happens. Runs on system work queue
 * (single-threaded, so no concurrent access).
 * ============================================================ */

static void refresh_work_handler(struct k_work *work) {
    if (!device_is_ready(strip)) {
        return;
    }

    /* --- Blink state (derived from tick counter) --- */

    blink_tick++;
    uint16_t charge_ticks = BLINK_CHARGE_MS / REFRESH_INTERVAL_MS;
    uint16_t critical_ticks = BLINK_CRITICAL_MS / REFRESH_INTERVAL_MS;
    bool is_critical = (!usb_powered && battery_level <= BATTERY_CRITICAL);

    if (usb_powered) {
        blink_on = (blink_tick % (charge_ticks * 2)) < charge_ticks;
    } else if (is_critical) {
        blink_on = (blink_tick % (critical_ticks * 2)) < critical_ticks;
    } else {
        blink_on = true;
    }

    /* --- Zone 1: Battery LEDs --- */

    int leds_active;
    if (BATTERY_LED_COUNT == 1) {
        leds_active = 1;
    } else {
        leds_active = (battery_level * BATTERY_LED_COUNT + 99) / 100;
        if (leds_active > BATTERY_LED_COUNT) {
            leds_active = BATTERY_LED_COUNT;
        }
    }

    bool should_blink = usb_powered || is_critical;
    bool show = !should_blink || blink_on;

    for (int i = 0; i < BATTERY_LED_COUNT; i++) {
        struct led_rgb color;
        uint8_t brightness;

        if (usb_powered) {
            color = BAT_COLOR_CHARGE;
            brightness = (i < leds_active)
                ? (show ? BATTERY_BLINK_BRIGHTNESS : BATTERY_INACTIVE_BRIGHTNESS)
                : BATTERY_INACTIVE_BRIGHTNESS;
        } else if (is_critical) {
            color = BAT_COLOR_CRIT;
            brightness = show ? BATTERY_BLINK_BRIGHTNESS : 0;
        } else {
            color = battery_gradient_color(i);
            brightness = (i < leds_active)
                ? BATTERY_ACTIVE_BRIGHTNESS
                : BATTERY_INACTIVE_BRIGHTNESS;
        }

        pixels[i] = scale_rgb(color, brightness);
    }

    /* --- Zone 2: Per-key LEDs --- */

    uint8_t layer = zmk_keymap_highest_layer_active();
    if (layer >= NUM_LAYER_COLORS) {
        layer = 0;
    }

    struct led_rgb base = layer_colors[layer];

    for (int i = 0; i < NUM_KEYS; i++) {
        uint8_t led_idx = key_to_led[i];
        if (key_pressed[i]) {
            pixels[led_idx].r = clamp_add(base.r, PRESS_BOOST);
            pixels[led_idx].g = clamp_add(base.g, PRESS_BOOST);
            pixels[led_idx].b = clamp_add(base.b, PRESS_BOOST);
        } else {
            pixels[led_idx] = base;
        }
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
 *
 * These only update state and schedule a refresh.
 * The actual pixel work happens in the work queue handler.
 * ============================================================ */

static int on_position_state_changed(const zmk_event_t *eh) {
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev && ev->position < NUM_KEYS) {
        key_pressed[ev->position] = ev->state;
    }
    return 0;
}

static int on_layer_state_changed(const zmk_event_t *eh) {
    /* Layer state is read fresh in the refresh handler via
     * zmk_keymap_highest_layer_active(), so nothing to do here. */
    return 0;
}

static int on_battery_state_changed(const zmk_event_t *eh) {
    struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev) {
        battery_level = ev->state_of_charge;
    }
    return 0;
}

static int on_usb_conn_state_changed(const zmk_event_t *eh) {
    struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (ev) {
        usb_powered = (ev->conn_state != ZMK_USB_CONN_NONE);
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

ZMK_LISTENER(led_driver_battery, on_battery_state_changed);
ZMK_SUBSCRIPTION(led_driver_battery, zmk_battery_state_changed);

ZMK_LISTENER(led_driver_usb, on_usb_conn_state_changed);
ZMK_SUBSCRIPTION(led_driver_usb, zmk_usb_conn_state_changed);

/* ============================================================
 * Initialization
 * ============================================================ */

static int led_driver_init(void) {
    strip = DEVICE_DT_GET(DT_NODELABEL(led_strip));
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    battery_level = zmk_battery_state_of_charge();
    usb_powered = zmk_usb_is_powered();

    /* Do the initial refresh synchronously (no contention at boot) */
    refresh_work_handler(NULL);

    /* Start periodic refresh timer (~30 FPS).
     * This decouples LED updates from the rapid event storm
     * that ARDUX combos generate, preventing flicker. */
    k_timer_start(&refresh_timer,
                  K_MSEC(REFRESH_INTERVAL_MS),
                  K_MSEC(REFRESH_INTERVAL_MS));

    LOG_INF("LED driver active: %d bat + %d key = %d LEDs, bat=%d%% usb=%d, refresh=%dms",
            BATTERY_LED_COUNT, NUM_KEY_LEDS, TOTAL_LEDS,
            battery_level, usb_powered, REFRESH_INTERVAL_MS);
    return 0;
}

SYS_INIT(led_driver_init, APPLICATION, 99);
