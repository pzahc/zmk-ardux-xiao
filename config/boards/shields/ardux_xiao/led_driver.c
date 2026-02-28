/*
 * LED driver for ARDUX XIAO
 *
 * Drives a single WS2812B strip with two zones:
 *
 * Zone 1: Per-key LEDs (indices 0-7)
 *   - Color reflects the active ARDUX layer
 *   - Brightens on keypress, dims on release
 *   - Strip routing (viewed from above):
 *       Top row:    LED7 ← LED6 ← LED5 ← LED4
 *       Bottom row: LED0 → LED1 → LED2 → LED3 ↑
 *
 * Zone 2: Battery indicator LEDs (indices 8+, configurable count)
 *   - Gradient from magenta (empty) to cyan (full)
 *   - Proportional fill: active LEDs are bright, inactive are very dim
 *   - Red-only blink when critically low (<10%)
 *   - Slow pulse when charging (cool white)
 *   - Works with 1 to N LEDs (set BATTERY_LED_COUNT)
 *
 * Color scheme: magenta / violet / cyan / coral / electric palette
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/battery.h>
#include <zmk/usb.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* ============================================================
 * Configuration — adjust these to match your hardware
 * ============================================================ */

/* Number of key LEDs (always 8 for ARDUX) */
#define NUM_KEYS 8
#define NUM_KEY_LEDS 8

/* Number of battery indicator LEDs (chained after key LEDs).
 * Set to 1 for a single status LED, or more for proportional bar.
 * Works for any value >= 1. */
#define BATTERY_LED_COUNT 5

/* Total LEDs on the strip */
#define TOTAL_LEDS (NUM_KEY_LEDS + BATTERY_LED_COUNT)

/* Battery LED brightness levels (0-255) */
#define BATTERY_ACTIVE_BRIGHTNESS 35   /* Lit LEDs (charged portion) */
#define BATTERY_INACTIVE_BRIGHTNESS 4  /* Unlit LEDs (dim ghost) */
#define BATTERY_BLINK_BRIGHTNESS 60    /* Critical/charging blink peak */

/* Blink intervals (ms) */
#define BLINK_CHARGE_MS 1500   /* Slow pulse when charging */
#define BLINK_CRITICAL_MS 400  /* Fast blink when critically low */

/* Battery thresholds (percentage) */
#define BATTERY_CRITICAL 10

/* Keypress brightness boost */
#define PRESS_BOOST 90

/* ============================================================
 * Key-to-LED mapping
 * ============================================================ */

static const uint8_t key_to_led[NUM_KEYS] = {
    7, /* pos 0 → LED 7 (top-left) */
    6, /* pos 1 → LED 6 */
    5, /* pos 2 → LED 5 */
    4, /* pos 3 → LED 4 (top-right) */
    0, /* pos 4 → LED 0 (bottom-left) */
    1, /* pos 5 → LED 1 */
    2, /* pos 6 → LED 2 */
    3, /* pos 7 → LED 3 (bottom-right) */
};

/* ============================================================
 * Layer colors — fun magenta/cyan/violet palette
 *
 * ARDUX layer IDs from ardux.dtsi:
 *   0 = Base, 1 = Numbers, 2 = Symbols, 3 = Parentheticals,
 *   4 = Navigation, 5 = BT Select, 6 = Mouse, 7 = Custom,
 *   8 = Big Sym, 9 = Big Function
 * ============================================================ */

#define NUM_LAYER_COLORS 10

static const struct led_rgb layer_colors[NUM_LAYER_COLORS] = {
    [0] = { .r = 10, .g = 30, .b = 35 }, /* Base:           teal */
    [1] = { .r = 15, .g = 8,  .b = 45 }, /* Numbers:        deep violet */
    [2] = { .r = 40, .g = 0,  .b = 35 }, /* Symbols:        magenta */
    [3] = { .r = 45, .g = 15, .b = 10 }, /* Parentheticals: coral */
    [4] = { .r = 0,  .g = 25, .b = 50 }, /* Navigation:     electric blue */
    [5] = { .r = 45, .g = 0,  .b = 45 }, /* BT Select:      hot pink */
    [6] = { .r = 40, .g = 30, .b = 0  }, /* Mouse:          amber */
    [7] = { .r = 20, .g = 40, .b = 5  }, /* Custom:         lime */
    [8] = { .r = 25, .g = 10, .b = 40 }, /* Big Sym:        lavender */
    [9] = { .r = 0,  .g = 35, .b = 30 }, /* Big Function:   aquamarine */
};

/* ============================================================
 * State
 * ============================================================ */

static struct led_rgb pixels[TOTAL_LEDS];
static bool key_pressed[NUM_KEYS];
static const struct device *strip;

/* Battery state */
static uint8_t battery_level = 100;  /* 0-100 */
static bool usb_powered = false;     /* USB connected = charging */
static bool blink_on = true;         /* Blink toggle state */

/* Timer for battery LED blink */
static void blink_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(blink_timer, blink_timer_handler, NULL);

/* ============================================================
 * Helpers
 * ============================================================ */

static inline uint8_t clamp_add(uint8_t a, uint8_t b) {
    uint16_t sum = (uint16_t)a + (uint16_t)b;
    return (sum > 255) ? 255 : (uint8_t)sum;
}

/* Scale an led_rgb by a brightness factor (0-255) */
static inline struct led_rgb scale_rgb(struct led_rgb c, uint8_t brightness) {
    return (struct led_rgb){
        .r = (uint8_t)(((uint16_t)c.r * brightness) / 255),
        .g = (uint8_t)(((uint16_t)c.g * brightness) / 255),
        .b = (uint8_t)(((uint16_t)c.b * brightness) / 255),
    };
}

/* Linear interpolation between two colors. t=0 gives 'a', t=255 gives 'b' */
static inline struct led_rgb lerp_rgb(struct led_rgb a, struct led_rgb b, uint8_t t) {
    return (struct led_rgb){
        .r = (uint8_t)(((uint16_t)a.r * (255 - t) + (uint16_t)b.r * t) / 255),
        .g = (uint8_t)(((uint16_t)a.g * (255 - t) + (uint16_t)b.g * t) / 255),
        .b = (uint8_t)(((uint16_t)a.b * (255 - t) + (uint16_t)b.b * t) / 255),
    };
}

/* ============================================================
 * Battery LED color logic
 *
 * Gradient from magenta (empty) → cyan (full)
 * Each LED position gets its own color on the gradient.
 * Critical (<10%) overrides to red.
 * Charging shows cool white pulse.
 * ============================================================ */

/* Magenta end (0% charge) */
static const struct led_rgb BAT_COLOR_EMPTY  = { .r = 255, .g = 0,   .b = 200 };
/* Cyan end (100% charge) */
static const struct led_rgb BAT_COLOR_FULL   = { .r = 0,   .g = 255, .b = 220 };
/* Critical override */
static const struct led_rgb BAT_COLOR_CRIT   = { .r = 255, .g = 0,   .b = 0   };
/* Charging pulse color (cool white with slight cyan tint) */
static const struct led_rgb BAT_COLOR_CHARGE = { .r = 140, .g = 220, .b = 255 };

/*
 * Get the gradient color for a specific LED position in the battery bar.
 * LED 0 (first) = empty end (magenta), LED N-1 (last) = full end (cyan).
 */
static struct led_rgb battery_gradient_color(int led_index) {
    if (BATTERY_LED_COUNT <= 1) {
        /* Single LED: color based on battery_level */
        return lerp_rgb(BAT_COLOR_EMPTY, BAT_COLOR_FULL, (uint8_t)((battery_level * 255) / 100));
    }
    /* Position on gradient: 0 for first LED, 255 for last */
    uint8_t t = (uint8_t)(((uint16_t)led_index * 255) / (BATTERY_LED_COUNT - 1));
    return lerp_rgb(BAT_COLOR_EMPTY, BAT_COLOR_FULL, t);
}

/* ============================================================
 * Refresh the entire strip
 * ============================================================ */

static void refresh_strip(void) {
    if (!strip) {
        return;
    }

    /* --- Zone 1: Per-key LEDs (0 to NUM_KEY_LEDS-1) --- */
    zmk_keymap_layer_index_t layer = zmk_keymap_highest_layer_active();
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

    /* --- Zone 2: Battery LEDs (NUM_KEY_LEDS to TOTAL_LEDS-1) --- */

    /*
     * How many LEDs represent the charged portion.
     * Round up so even 1% shows at least 1 active LED.
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

    bool is_critical = (!usb_powered && battery_level <= BATTERY_CRITICAL);
    bool should_blink = usb_powered || is_critical;
    bool show = !should_blink || blink_on;

    for (int i = 0; i < BATTERY_LED_COUNT; i++) {
        int led_idx = NUM_KEY_LEDS + i;
        struct led_rgb color;
        uint8_t brightness;

        if (usb_powered) {
            /* Charging: all LEDs pulse cool white up to the charged level */
            color = BAT_COLOR_CHARGE;
            if (i < leds_active) {
                brightness = show ? BATTERY_BLINK_BRIGHTNESS : BATTERY_INACTIVE_BRIGHTNESS;
            } else {
                brightness = BATTERY_INACTIVE_BRIGHTNESS;
            }
        } else if (is_critical) {
            /* Critical: all LEDs flash red */
            color = BAT_COLOR_CRIT;
            brightness = show ? BATTERY_BLINK_BRIGHTNESS : 0;
        } else {
            /* Normal: gradient colors, active LEDs bright, inactive LEDs dim ghost */
            color = battery_gradient_color(i);
            if (i < leds_active) {
                brightness = BATTERY_ACTIVE_BRIGHTNESS;
            } else {
                brightness = BATTERY_INACTIVE_BRIGHTNESS;
            }
        }

        pixels[led_idx] = scale_rgb(color, brightness);
    }

    /* Push to hardware */
    int rc = led_strip_update_rgb(strip, pixels, TOTAL_LEDS);
    if (rc) {
        LOG_ERR("Failed to update LED strip: %d", rc);
    }
}

/* ============================================================
 * Blink timer
 * ============================================================ */

static void update_blink_timer(void) {
    bool need_blink = usb_powered || (battery_level <= BATTERY_CRITICAL);

    if (need_blink) {
        int interval = usb_powered ? BLINK_CHARGE_MS : BLINK_CRITICAL_MS;
        k_timer_start(&blink_timer, K_MSEC(interval), K_MSEC(interval));
    } else {
        k_timer_stop(&blink_timer);
        blink_on = true;
    }
}

static void blink_timer_handler(struct k_timer *timer) {
    blink_on = !blink_on;
    refresh_strip();
}

/* ============================================================
 * ZMK Event Handlers
 * ============================================================ */

static int on_position_state_changed(const zmk_event_t *ev) {
    struct zmk_position_state_changed *pos_ev = as_zmk_position_state_changed(ev);
    if (pos_ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (pos_ev->position < NUM_KEYS) {
        key_pressed[pos_ev->position] = pos_ev->state;
        refresh_strip();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

static int on_layer_state_changed(const zmk_event_t *ev) {
    refresh_strip();
    return ZMK_EV_EVENT_BUBBLE;
}

static int on_battery_state_changed(const zmk_event_t *ev) {
    struct zmk_battery_state_changed *bat_ev = as_zmk_battery_state_changed(ev);
    if (bat_ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    battery_level = bat_ev->state_of_charge;
    update_blink_timer();
    refresh_strip();
    return ZMK_EV_EVENT_BUBBLE;
}

static int on_usb_conn_state_changed(const zmk_event_t *ev) {
    struct zmk_usb_conn_state_changed *usb_ev = as_zmk_usb_conn_state_changed(ev);
    if (usb_ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    usb_powered = (usb_ev->conn_state != ZMK_USB_CONN_NONE);
    update_blink_timer();
    refresh_strip();
    return ZMK_EV_EVENT_BUBBLE;
}

/* ============================================================
 * Event subscriptions
 * ============================================================ */

ZMK_LISTENER(led_driver_pos, on_position_state_changed);
ZMK_SUBSCRIPTION(led_driver_pos, zmk_position_state_changed);

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
        strip = NULL;
        return -ENODEV;
    }

    /* Read initial state */
    battery_level = zmk_battery_state_of_charge();
    usb_powered = zmk_usb_is_powered();

    LOG_INF("LED driver initialized (%d key LEDs + %d battery LEDs = %d total)",
            NUM_KEY_LEDS, BATTERY_LED_COUNT, TOTAL_LEDS);

    update_blink_timer();
    refresh_strip();
    return 0;
}

SYS_INIT(led_driver_init, APPLICATION, 99);
