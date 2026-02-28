/*
 * LED DIAGNOSTIC MODE — EXHAUSTIVE
 *
 * Loops forever through tests on boot. Never enters normal mode.
 *
 * Onboard LED announces each test:
 *   RED blinks    = Group 1: Pin & hardware checks
 *   GREEN blinks  = Group 2: Basic strip colors
 *   BLUE blinks   = Group 3: Patterns & single LEDs
 *   WHITE blinks  = Group 4: Edge cases & inversion
 *   MAGENTA blinks= Group 5: Chain length & timing
 *   CYAN blinks   = Group 6: Color mapping
 *
 * Blink count = test number within group.
 *
 * WATCH FOR:
 *   - Multimeter on D9 during pin tests (Group 1)
 *   - Any LED strip illumination during any test
 *   - If strip lights on "ALL OFF" test = signal inverted
 *   - If only short-chain tests work = buffer/timing issue
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(led_diag, 4);

#define TOTAL_LEDS 13
#define TEST_HOLD_MS 3000

static struct led_rgb pixels[TOTAL_LEDS];
static const struct device *strip;

/* ============================================================
 * Onboard RGB LED (active-low on XIAO nRF52840 Sense)
 *   Red=P0.26  Green=P0.30  Blue=P0.06
 * ============================================================ */

static const struct device *gpio0;
static const struct device *gpio1;

#define PIN_RED   26
#define PIN_GREEN 30
#define PIN_BLUE   6

static void onboard_init(void) {
    gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    gpio_pin_configure(gpio0, PIN_RED,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_GREEN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_BLUE,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_set(gpio0, PIN_RED,   1);
    gpio_pin_set(gpio0, PIN_GREEN, 1);
    gpio_pin_set(gpio0, PIN_BLUE,  1);
}

static void onboard_set(int r, int g, int b) {
    gpio_pin_set(gpio0, PIN_RED,   r ? 0 : 1);
    gpio_pin_set(gpio0, PIN_GREEN, g ? 0 : 1);
    gpio_pin_set(gpio0, PIN_BLUE,  b ? 0 : 1);
}

static void onboard_off(void) { onboard_set(0, 0, 0); }

static void onboard_blink(int r, int g, int b, int count, int ms) {
    for (int i = 0; i < count; i++) {
        onboard_set(r, g, b);
        k_msleep(ms);
        onboard_off();
        k_msleep(ms);
    }
}

/* ============================================================
 * Strip helpers
 * ============================================================ */

static void strip_clear(void) { memset(pixels, 0, sizeof(pixels)); }

static void strip_fill(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < TOTAL_LEDS; i++) {
        pixels[i].r = r; pixels[i].g = g; pixels[i].b = b;
    }
}

static void strip_set(int idx, uint8_t r, uint8_t g, uint8_t b) {
    if (idx >= 0 && idx < TOTAL_LEDS) {
        pixels[idx].r = r; pixels[idx].g = g; pixels[idx].b = b;
    }
}

static int strip_send(void) {
    int rc = led_strip_update_rgb(strip, pixels, TOTAL_LEDS);
    if (rc) { LOG_ERR("strip_send: %d", rc); }
    return rc;
}

static int strip_send_n(int n) {
    int rc = led_strip_update_rgb(strip, pixels, n);
    if (rc) { LOG_ERR("strip_send_n(%d): %d", n, rc); }
    return rc;
}

static void announce(int r, int g, int b, int num) {
    LOG_INF("──────────────────────────────────");
    onboard_off();
    k_msleep(400);
    onboard_blink(r, g, b, num, 200);
    k_msleep(200);
    onboard_set(r, g, b);
}

/* ============================================================
 * GPIO pin toggle test — toggles a pin as raw GPIO output
 * so you can probe with multimeter. Toggles for ~2 seconds.
 * Returns 0 if gpio_pin_configure succeeded, negative on error.
 * ============================================================ */

static int test_gpio_toggle(const struct device *port, gpio_pin_t pin,
                            int duration_ms, const char *label) {
    int rc = gpio_pin_configure(port, pin, GPIO_OUTPUT);
    if (rc) {
        LOG_ERR("GPIO config %s (pin %d): FAILED rc=%d", label, pin, rc);
        return rc;
    }
    LOG_INF("GPIO toggle %s (pin %d): toggling for %d ms", label, pin, duration_ms);

    int64_t end = k_uptime_get() + duration_ms;
    int cycles = 0;
    while (k_uptime_get() < end) {
        gpio_pin_set(port, pin, 1);
        k_busy_wait(500);  /* 500us high */
        gpio_pin_set(port, pin, 0);
        k_busy_wait(500);  /* 500us low */
        cycles++;
    }
    LOG_INF("GPIO toggle %s: %d cycles completed", label, cycles);

    /* Leave pin LOW */
    gpio_pin_set(port, pin, 0);
    return 0;
}

/* ============================================================
 * THE DIAGNOSTIC TESTS
 * ============================================================ */

static void run_diagnostics(void) {
    int rc;

    /* ================================================================
     * GROUP 1: PIN & HARDWARE CHECKS (onboard = RED)
     *
     * Use multimeter on D8 (P1.13), D9 (P1.14), D10 (P1.15)
     * Expected: ~1.6V average during toggle (rapid square wave)
     * Then ~0V when done.
     * ================================================================ */

    /* Test 1.1: Toggle D9 (P1.14) as raw GPIO — BEFORE I2S touches it
     * This proves: can the MCU physically drive this pin?
     * Measure D9 with multimeter — should see ~1.6V during this test.
     * If you see 3.5V (floating), the pin is not being driven. */
    announce(1, 0, 0, 1);
    LOG_INF("T1.1: GPIO toggle D9/P1.14 — measure with multimeter!");
    rc = test_gpio_toggle(gpio1, 14, 3000, "D9/P1.14");
    LOG_INF("T1.1: result=%d (0=OK)", rc);
    k_msleep(500);

    /* Test 1.2: Toggle D8 (P1.13) as raw GPIO
     * Verifies pin mapping — probe D8 pad with multimeter */
    announce(1, 0, 0, 2);
    LOG_INF("T1.2: GPIO toggle D8/P1.13");
    rc = test_gpio_toggle(gpio1, 13, 2000, "D8/P1.13");
    LOG_INF("T1.2: result=%d", rc);
    k_msleep(500);

    /* Test 1.3: Toggle D10 (P1.15) as raw GPIO */
    announce(1, 0, 0, 3);
    LOG_INF("T1.3: GPIO toggle D10/P1.15");
    rc = test_gpio_toggle(gpio1, 15, 2000, "D10/P1.15");
    LOG_INF("T1.3: result=%d", rc);
    k_msleep(500);

    /* Test 1.4: Check DT node existence and strip device readiness */
    announce(1, 0, 0, 4);
    LOG_INF("T1.4: Device tree & driver check");
#if DT_NODE_EXISTS(DT_NODELABEL(led_strip))
    LOG_INF("  DT node 'led_strip': EXISTS");
#else
    LOG_ERR("  DT node 'led_strip': MISSING — overlay not applied!");
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(i2s_led))
    LOG_INF("  DT node 'i2s_led': EXISTS");
#else
    LOG_ERR("  DT node 'i2s_led': MISSING");
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(i2s0))
    LOG_INF("  DT node 'i2s0': EXISTS");
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2s0), okay)
    LOG_INF("  DT node 'i2s0': status=okay");
#else
    LOG_ERR("  DT node 'i2s0': status != okay!");
#endif
#else
    LOG_ERR("  DT node 'i2s0': MISSING");
#endif
    LOG_INF("  strip device_is_ready: %s", device_is_ready(strip) ? "YES" : "NO");
    LOG_INF("  strip device name: %s", strip ? strip->name : "(null)");
    k_msleep(TEST_HOLD_MS);

    /* Test 1.5: Try to reconfigure P1.14 as GPIO AFTER I2S init.
     * If I2S properly claimed the pin, this might fail or conflict. */
    announce(1, 0, 0, 5);
    LOG_INF("T1.5: GPIO on P1.14 AFTER I2S init (conflict test)");
    rc = gpio_pin_configure(gpio1, 14, GPIO_OUTPUT);
    LOG_INF("  gpio_pin_configure returned: %d (0=success, neg=I2S may have claimed it)", rc);
    if (rc == 0) {
        LOG_WRN("  Pin NOT claimed by I2S! This means I2S is NOT driving D9.");
        /* Toggle it to show it's driveable as GPIO even now */
        for (int i = 0; i < 2000; i++) {
            gpio_pin_set(gpio1, 14, 1);
            k_busy_wait(500);
            gpio_pin_set(gpio1, 14, 0);
            k_busy_wait(500);
        }
    } else {
        LOG_INF("  Pin claimed by I2S — good, I2S peripheral owns the pin.");
    }
    k_msleep(TEST_HOLD_MS);

    /* Test 1.6: High drive strength on P1.14
     * nRF52840 defaults to standard drive. DS_ALT flag requests high drive.
     * More current = better chance of reaching WS2812 VIH threshold at 5V. */
    announce(1, 0, 0, 6);
    LOG_INF("T1.6: GPIO toggle P1.14 with high drive strength");
    /* Try with drive-strength-related extra flags. On nRF, the pinctrl
     * driver handles drive strength, but we can at least toggle fast. */
    rc = gpio_pin_configure(gpio1, 14, GPIO_OUTPUT);
    if (rc == 0) {
        LOG_INF("  Toggling D9 for 2s (fast square wave)");
        int64_t end = k_uptime_get() + 2000;
        while (k_uptime_get() < end) {
            gpio_pin_set(gpio1, 14, 1);
            k_busy_wait(100);  /* 100us per half = 5kHz — visible on scope */
            gpio_pin_set(gpio1, 14, 0);
            k_busy_wait(100);
        }
        gpio_pin_set(gpio1, 14, 0);
    } else {
        LOG_ERR("  GPIO config failed: %d", rc);
    }
    k_msleep(1000);

    /* ================================================================
     * GROUP 2: BASIC STRIP COLORS (onboard = GREEN)
     *
     * If none of these light the strip, it's not a color/data issue.
     * ================================================================ */

    /* Test 2.1: All WHITE max brightness */
    announce(0, 1, 0, 1);
    LOG_INF("T2.1: All WHITE (255,255,255)");
    strip_fill(255, 255, 255);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 2.2: All RED */
    announce(0, 1, 0, 2);
    LOG_INF("T2.2: All RED (255,0,0)");
    strip_fill(255, 0, 0);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 2.3: All GREEN */
    announce(0, 1, 0, 3);
    LOG_INF("T2.3: All GREEN (0,255,0)");
    strip_fill(0, 255, 0);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 2.4: All BLUE */
    announce(0, 1, 0, 4);
    LOG_INF("T2.4: All BLUE (0,0,255)");
    strip_fill(0, 0, 255);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 2.5: Multiple consecutive sends — some strips drop first frame */
    announce(0, 1, 0, 5);
    LOG_INF("T2.5: 5x consecutive sends, all WHITE, 100ms apart");
    strip_fill(255, 255, 255);
    for (int i = 0; i < 5; i++) {
        rc = strip_send();
        LOG_INF("  send %d: rc=%d", i + 1, rc);
        k_msleep(100);
    }
    k_msleep(TEST_HOLD_MS);

    /* Test 2.6: Send with long reset delay between frames */
    announce(0, 1, 0, 6);
    LOG_INF("T2.6: Send WHITE, wait 1s, send WHITE again (long reset)");
    strip_fill(255, 255, 255);
    strip_send();
    k_msleep(1000);  /* Long reset */
    strip_send();
    k_msleep(TEST_HOLD_MS);

    /* ================================================================
     * GROUP 3: PATTERNS & SINGLE LEDS (onboard = BLUE)
     * ================================================================ */

    /* Test 3.1: Walking LED through all positions */
    announce(0, 0, 1, 1);
    LOG_INF("T3.1: Walking LED (single white, all positions)");
    for (int i = 0; i < TOTAL_LEDS; i++) {
        strip_clear();
        strip_set(i, 255, 255, 255);
        strip_send();
        k_msleep(400);
    }
    k_msleep(500);

    /* Test 3.2: First LED only */
    announce(0, 0, 1, 2);
    LOG_INF("T3.2: LED[0] only = WHITE");
    strip_clear();
    strip_set(0, 255, 255, 255);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 3.3: Last LED only */
    announce(0, 0, 1, 3);
    LOG_INF("T3.3: LED[%d] only = WHITE", TOTAL_LEDS - 1);
    strip_clear();
    strip_set(TOTAL_LEDS - 1, 255, 255, 255);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 3.4: All LEDs different — R, G, B cycling */
    announce(0, 0, 1, 4);
    LOG_INF("T3.4: Each LED different color (R/G/B cycle)");
    strip_clear();
    for (int i = 0; i < TOTAL_LEDS; i++) {
        switch (i % 3) {
        case 0: strip_set(i, 255, 0, 0); break;
        case 1: strip_set(i, 0, 255, 0); break;
        case 2: strip_set(i, 0, 0, 255); break;
        }
    }
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* ================================================================
     * GROUP 4: EDGE CASES & INVERSION (onboard = WHITE)
     * ================================================================ */

    /* Test 4.1: ALL OFF — if strip lights here, signal is INVERTED */
    announce(1, 1, 1, 1);
    LOG_INF("T4.1: All OFF (0,0,0) — ** IF LIT = INVERTED SIGNAL **");
    strip_fill(0, 0, 0);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 4.2: Dim white (1,1,1) — minimum non-zero */
    announce(1, 1, 1, 2);
    LOG_INF("T4.2: Dim white (1,1,1)");
    strip_fill(1, 1, 1);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 4.3: Medium brightness (128,128,128) */
    announce(1, 1, 1, 3);
    LOG_INF("T4.3: Medium white (128,128,128)");
    strip_fill(128, 128, 128);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 4.4: Brightness ramp — LEDs go from 0 to 255 across the strip */
    announce(1, 1, 1, 4);
    LOG_INF("T4.4: Brightness ramp (0 to 255 across strip)");
    strip_clear();
    for (int i = 0; i < TOTAL_LEDS; i++) {
        uint8_t v = (uint8_t)((i * 255) / (TOTAL_LEDS - 1));
        strip_set(i, v, v, v);
    }
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* ================================================================
     * GROUP 5: CHAIN LENGTH & TIMING (onboard = MAGENTA)
     * ================================================================ */

    /* Test 5.1: chain=1, just LED[0] white */
    announce(1, 0, 1, 1);
    LOG_INF("T5.1: chain=1, LED[0]=WHITE");
    strip_clear();
    strip_set(0, 255, 255, 255);
    rc = strip_send_n(1);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 5.2: chain=2 */
    announce(1, 0, 1, 2);
    LOG_INF("T5.2: chain=2, LED[0..1]=WHITE");
    strip_clear();
    strip_set(0, 255, 255, 255);
    strip_set(1, 255, 255, 255);
    rc = strip_send_n(2);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 5.3: chain=3 */
    announce(1, 0, 1, 3);
    LOG_INF("T5.3: chain=3, all WHITE");
    strip_fill(255, 255, 255);
    rc = strip_send_n(3);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 5.4: chain=8 */
    announce(1, 0, 1, 4);
    LOG_INF("T5.4: chain=8, all WHITE");
    strip_fill(255, 255, 255);
    rc = strip_send_n(8);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 5.5: Rapid toggle — tests timing/latching */
    announce(1, 0, 1, 5);
    LOG_INF("T5.5: Rapid toggle white/off x10");
    for (int i = 0; i < 10; i++) {
        strip_fill(255, 255, 255);
        strip_send();
        k_msleep(200);
        strip_fill(0, 0, 0);
        strip_send();
        k_msleep(200);
    }

    /* ================================================================
     * GROUP 6: COLOR MAPPING (onboard = CYAN)
     * We label what we SEND via API. You report what you SEE.
     * If colors appear wrong, the GRB mapping in the overlay is off.
     * ================================================================ */

    /* Test 6.1: API R=255 — should appear RED */
    announce(0, 1, 1, 1);
    LOG_INF("T6.1: Sending R=255,G=0,B=0 — expect RED");
    strip_fill(255, 0, 0);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 6.2: API G=255 — should appear GREEN */
    announce(0, 1, 1, 2);
    LOG_INF("T6.2: Sending R=0,G=255,B=0 — expect GREEN");
    strip_fill(0, 255, 0);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 6.3: API B=255 — should appear BLUE */
    announce(0, 1, 1, 3);
    LOG_INF("T6.3: Sending R=0,G=0,B=255 — expect BLUE");
    strip_fill(0, 0, 255);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* ================================================================
     * END OF CYCLE
     * ================================================================ */
    LOG_INF("══════════════════════════════════");
    LOG_INF("  CYCLE COMPLETE — restarting");
    LOG_INF("══════════════════════════════════");
    onboard_off();
    for (int i = 0; i < 8; i++) {
        onboard_set(0, 1, 1);
        k_msleep(80);
        onboard_off();
        k_msleep(80);
    }
    k_msleep(1500);
}

/* ============================================================
 * Init
 * ============================================================ */

static int led_driver_init(void) {
    onboard_init();

    /* Announce boot: solid YELLOW 1.5s */
    onboard_set(1, 1, 0);
    LOG_INF("╔══════════════════════════════════╗");
    LOG_INF("║  LED DIAGNOSTIC MODE — STARTING  ║");
    LOG_INF("╚══════════════════════════════════╝");
    k_msleep(1500);
    onboard_off();

    /* Get the strip device */
    strip = DEVICE_DT_GET(DT_NODELABEL(led_strip));
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device NOT READY — looping red blink forever");
        LOG_ERR("This means the I2S or WS2812 driver failed to init.");
        while (1) {
            onboard_blink(1, 0, 0, 3, 150);
            k_msleep(500);
        }
        return -ENODEV;
    }

    LOG_INF("Strip device ready: name='%s'", strip->name);
    LOG_INF("Total LEDs configured: %d", TOTAL_LEDS);

    /* Check I2S device too */
#if DT_NODE_EXISTS(DT_NODELABEL(i2s_led))
    {
        const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_led));
        LOG_INF("I2S device ready: %s (name='%s')",
                device_is_ready(i2s_dev) ? "YES" : "NO",
                i2s_dev ? i2s_dev->name : "(null)");
    }
#else
    LOG_ERR("DT node 'i2s_led' does not exist!");
#endif

    /* Solid GREEN 1s = all good, entering test loop */
    onboard_set(0, 1, 0);
    k_msleep(1000);
    onboard_off();
    k_msleep(500);

    while (1) {
        run_diagnostics();
    }

    return 0;
}

SYS_INIT(led_driver_init, APPLICATION, 99);
