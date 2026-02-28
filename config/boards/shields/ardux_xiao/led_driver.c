/*
 * LED DIAGNOSTIC MODE — SPI + PIN IDENTITY
 *
 * Loops forever through tests on boot. Never enters normal mode.
 *
 * ===== HOW TO USE =====
 *
 * Multimeter on DC voltage. Probe the XIAO pads as indicated.
 *
 * SPI-dependent tests run FIRST (while pin config is pristine from boot).
 * GPIO pin identity test runs LAST (may alter pin config for SPI3 MOSI).
 *
 * GROUP 1 — RAW SPI (onboard = YELLOW, blinks 1-7)
 *   Bypasses the WS2812 driver entirely. Calls spi_write() directly
 *   on &spi3 with manually constructed spi_config. Tests:
 *   1.1: SPI3 device readiness
 *   1.2: Raw 0xFF bytes (all ones — MOSI should be HIGH the whole time)
 *   1.3: Raw 0xAA bytes (alternating — MOSI toggles, multimeter ~1.6V)
 *   1.4: Raw 0x00 bytes (all zeros — MOSI stays LOW)
 *   1.5: SPI mode 1 (CPHA=1)
 *   1.6: SPI mode 2 (CPOL=1)
 *   1.7: SPI mode 3 (CPOL=1, CPHA=1)
 *   If D9 shows voltage during 1.2/1.3 but not 1.4 → SPI3 works!
 *   If D9 shows nothing on any test → SPI3 is NOT driving the pin.
 *
 * GROUP 2 — RAW WS2812 FRAME (onboard = CYAN, blinks 1-5)
 *   Manually encodes WS2812 pixel data using one_frame/zero_frame
 *   and sends via raw spi_write(). Bypasses ws2812 driver but uses
 *   the same encoding. Tests if our manual encoding lights the strip.
 *   2.1: 1 LED all-white, standard frames (0x70/0x40)
 *   2.2: 1 LED all-white, 10x rapid sends (in case first frame drops)
 *   2.3: 1 LED, wider one/zero frames (0x7C/0x60)
 *   2.4: 1 LED, even wider one frame (0x7E/0x40)
 *   2.5: 1 LED RED only (simpler pattern — fewer one pulses)
 *
 * GROUP 3 — WS2812 DRIVER STRIP TESTS (onboard = GREEN, blinks 1-6)
 *   Uses led_strip_update_rgb() through the WS2812 SPI driver.
 *   Standard color tests: white, red, green, blue, multi-send, reset.
 *
 * GROUP 4 — PATTERNS (onboard = BLUE, blinks 1-4)
 *   Walking LED, first LED, RGB cycle, all-off inversion check.
 *
 * GROUP 5 — CHAIN LENGTH (onboard = MAGENTA, blinks 1-5)
 *   chain=1, 2, 3, 8, 13.
 *
 * GROUP 6 — PIN IDENTITY (onboard = RED, blinks 1/2/3) *** RUNS LAST ***
 *   Toggles D8, D9, D10 as raw GPIO, one at a time.
 *   1 blink=D8, 2=D9, 3=D10. ~1kHz square wave for 3s each.
 *   Multimeter should read ~1.6V on the toggling pad.
 *
 * After all groups: rapid cyan blink = cycle complete, restarting.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(led_diag, 4);

#define TOTAL_LEDS 13
#define TEST_HOLD_MS 3000
#define PIN_TOGGLE_MS 3000

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
 * Strip helpers (for WS2812 driver tests)
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

/* Announce a test */
static void announce(int r, int g, int b, int num) {
    LOG_INF("──────────────────────────────────");
    onboard_off();
    k_msleep(400);
    onboard_blink(r, g, b, num, 200);
    k_msleep(200);
    onboard_set(r, g, b);
}

/* ============================================================
 * GPIO pin toggle test
 * ============================================================ */

static int test_gpio_toggle(const struct device *port, gpio_pin_t pin,
                            int duration_ms, const char *label) {
    int rc = gpio_pin_configure(port, pin, GPIO_OUTPUT);
    if (rc) {
        LOG_ERR("GPIO config %s (pin %d): FAILED rc=%d", label, pin, rc);
        return rc;
    }
    LOG_INF("GPIO toggle %s (pin %d): toggling for %d ms",
            label, pin, duration_ms);

    int64_t end = k_uptime_get() + duration_ms;
    int cycles = 0;
    while (k_uptime_get() < end) {
        gpio_pin_set(port, pin, 1);
        k_busy_wait(500);
        gpio_pin_set(port, pin, 0);
        k_busy_wait(500);
        cycles++;
    }
    LOG_INF("GPIO toggle %s: %d cycles completed", label, cycles);
    gpio_pin_set(port, pin, 0);
    return 0;
}

/* ============================================================
 * Pin table: D8, D9, D10 only
 * ============================================================ */

struct test_pin {
    const struct device **port;
    gpio_pin_t pin;
    const char *label;
    int announce_count;
};

#define SPI_PIN_COUNT 3
static struct test_pin spi_pins[SPI_PIN_COUNT];

static void init_pin_table(void) {
    spi_pins[0] = (struct test_pin){ &gpio1, 13, "D8/P1.13",  1 };
    spi_pins[1] = (struct test_pin){ &gpio1, 14, "D9/P1.14",  2 };
    spi_pins[2] = (struct test_pin){ &gpio1, 15, "D10/P1.15", 3 };
}

/* ============================================================
 * Raw SPI helpers
 *
 * Send arbitrary bytes directly on &spi3, bypassing the WS2812
 * driver. This tests whether the SPI peripheral is actually
 * clocking data out on the MOSI pin (D9/P1.14).
 * ============================================================ */

/* Buffer for raw SPI tests — big enough for 1 LED of WS2812 data
 * (3 colors * 8 SPI bytes per color = 24 bytes) plus some extra */
#define RAW_SPI_BUF_SIZE 64
static uint8_t raw_spi_buf[RAW_SPI_BUF_SIZE];

/*
 * Send raw bytes on SPI3 with given operation flags.
 * Returns the spi_write() return code.
 */
static int raw_spi_send(const struct device *spi_dev, uint16_t operation,
                        uint8_t *buf, size_t len, const char *desc) {
    struct spi_config cfg = {
        .frequency = 4000000,  /* 4 MHz — same as overlay */
        .operation = operation,
        .slave = 0,
        .cs = { .gpio = { .port = NULL }, .delay = 0 },
    };

    struct spi_buf spi_buf = { .buf = buf, .len = len };
    struct spi_buf_set tx = { .buffers = &spi_buf, .count = 1 };

    int rc = spi_write(spi_dev, &cfg, &tx);
    LOG_INF("  raw_spi_send(%s): len=%d rc=%d", desc, (int)len, rc);
    return rc;
}

/* Standard SPI operation: mode 0, 8-bit, MSB first (matches WS2812 driver) */
#define SPI_OP_MODE0  (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8))
/* Mode 1: CPHA=1 */
#define SPI_OP_MODE1  (SPI_OP_MODE0 | SPI_MODE_CPHA)
/* Mode 2: CPOL=1 */
#define SPI_OP_MODE2  (SPI_OP_MODE0 | SPI_MODE_CPOL)
/* Mode 3: CPOL=1, CPHA=1 */
#define SPI_OP_MODE3  (SPI_OP_MODE0 | SPI_MODE_CPOL | SPI_MODE_CPHA)

/*
 * Manually encode a single WS2812 color byte into 8 SPI bytes,
 * using the given one_frame and zero_frame values.
 * Same algorithm as ws2812_spi.c ws2812_spi_ser().
 */
static void encode_ws2812_byte(uint8_t *out, uint8_t color,
                               uint8_t one_frame, uint8_t zero_frame) {
    for (int i = 0; i < 8; i++) {
        out[i] = (color & (0x80 >> i)) ? one_frame : zero_frame;
    }
}

/* ============================================================
 * THE DIAGNOSTIC TESTS
 * ============================================================ */

static void run_diagnostics(void) {
    int rc;
    const struct device *spi3_dev = NULL;

    /* Get SPI3 device for raw SPI tests */
#if DT_NODE_EXISTS(DT_NODELABEL(spi3))
    spi3_dev = DEVICE_DT_GET(DT_NODELABEL(spi3));
#endif

    /*
     * NOTE ON TEST ORDER:
     * SPI-dependent tests (Groups 1-4) run FIRST while the SPI3 pin
     * config is pristine from boot. The GPIO pin identity test (Group 5)
     * runs LAST because calling gpio_pin_configure() on D9/P1.14 could
     * potentially interfere with SPI3's MOSI pin assignment.
     * On nRF52840, the SPIM peripheral should override GPIO when active,
     * so it's likely harmless — but better safe than sorry.
     */

    /* ================================================================
     * GROUP 1: RAW SPI TESTS (onboard = YELLOW)
     *
     * Bypasses WS2812 driver entirely. Sends raw bytes on SPI3.
     * Probe D9 with multimeter during each test.
     *
     * If D9 shows voltage on 0xFF / 0xAA tests but not 0x00 → SPI works.
     * If D9 shows nothing → SPI3 is not driving the MOSI pin at all.
     * ================================================================ */

    LOG_INF("════════════════════════════════════");
    LOG_INF("  GROUP 1: RAW SPI (bypass WS2812 driver)");
    LOG_INF("════════════════════════════════════");

    /* Test 1.1: SPI3 device check */
    announce(1, 1, 0, 1);  /* yellow = red+green */
    LOG_INF("T1.1: SPI3 device check");
    if (spi3_dev == NULL) {
        LOG_ERR("  SPI3 device: NOT FOUND in device tree!");
        LOG_ERR("  Cannot run raw SPI tests.");
        k_msleep(TEST_HOLD_MS);
    } else {
        LOG_INF("  SPI3 device: name='%s'", spi3_dev->name);
        LOG_INF("  SPI3 ready: %s", device_is_ready(spi3_dev) ? "YES" : "NO");
        if (!device_is_ready(spi3_dev)) {
            LOG_ERR("  SPI3 NOT READY — raw SPI tests will likely fail");
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 1.2: Raw 0xFF — all ones, MOSI should be HIGH entire transfer.
         * With 64 bytes at 4MHz, transfer lasts ~128us.
         * Multimeter might just barely catch it. Sending 10x with gaps. */
        announce(1, 1, 0, 2);
        LOG_INF("T1.2: Raw SPI 0xFF x64 bytes, 10 sends, mode 0");
        LOG_INF("  Probe D9 — should see brief voltage if SPI drives MOSI");
        memset(raw_spi_buf, 0xFF, RAW_SPI_BUF_SIZE);
        for (int i = 0; i < 10; i++) {
            rc = raw_spi_send(spi3_dev, SPI_OP_MODE0, raw_spi_buf,
                              RAW_SPI_BUF_SIZE, "0xFF mode0");
            k_msleep(50);
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 1.3: Raw 0xAA — alternating bits, MOSI toggles.
         * Multimeter should read ~1.6V average. */
        announce(1, 1, 0, 3);
        LOG_INF("T1.3: Raw SPI 0xAA x64 bytes, 10 sends, mode 0");
        LOG_INF("  Probe D9 — alternating pattern, ~1.6V avg");
        memset(raw_spi_buf, 0xAA, RAW_SPI_BUF_SIZE);
        for (int i = 0; i < 10; i++) {
            rc = raw_spi_send(spi3_dev, SPI_OP_MODE0, raw_spi_buf,
                              RAW_SPI_BUF_SIZE, "0xAA mode0");
            k_msleep(50);
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 1.4: Raw 0x00 — all zeros, MOSI should stay LOW.
         * If multimeter shows voltage here → signal is inverted! */
        announce(1, 1, 0, 4);
        LOG_INF("T1.4: Raw SPI 0x00 x64 bytes, mode 0");
        LOG_INF("  Probe D9 — should see ~0V (MOSI LOW)");
        LOG_INF("  ** IF voltage here → SIGNAL INVERTED **");
        memset(raw_spi_buf, 0x00, RAW_SPI_BUF_SIZE);
        for (int i = 0; i < 10; i++) {
            rc = raw_spi_send(spi3_dev, SPI_OP_MODE0, raw_spi_buf,
                              RAW_SPI_BUF_SIZE, "0x00 mode0");
            k_msleep(50);
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 1.5: Mode 1 (CPHA=1) — same 0xAA data */
        announce(1, 1, 0, 5);
        LOG_INF("T1.5: Raw SPI 0xAA, MODE 1 (CPHA=1)");
        memset(raw_spi_buf, 0xAA, RAW_SPI_BUF_SIZE);
        for (int i = 0; i < 10; i++) {
            rc = raw_spi_send(spi3_dev, SPI_OP_MODE1, raw_spi_buf,
                              RAW_SPI_BUF_SIZE, "0xAA mode1");
            k_msleep(50);
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 1.6: Mode 2 (CPOL=1) — same 0xAA data */
        announce(1, 1, 0, 6);
        LOG_INF("T1.6: Raw SPI 0xAA, MODE 2 (CPOL=1)");
        memset(raw_spi_buf, 0xAA, RAW_SPI_BUF_SIZE);
        for (int i = 0; i < 10; i++) {
            rc = raw_spi_send(spi3_dev, SPI_OP_MODE2, raw_spi_buf,
                              RAW_SPI_BUF_SIZE, "0xAA mode2");
            k_msleep(50);
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 1.7: Mode 3 (CPOL=1, CPHA=1) — same 0xAA data */
        announce(1, 1, 0, 7);
        LOG_INF("T1.7: Raw SPI 0xAA, MODE 3 (CPOL=1,CPHA=1)");
        memset(raw_spi_buf, 0xAA, RAW_SPI_BUF_SIZE);
        for (int i = 0; i < 10; i++) {
            rc = raw_spi_send(spi3_dev, SPI_OP_MODE3, raw_spi_buf,
                              RAW_SPI_BUF_SIZE, "0xAA mode3");
            k_msleep(50);
        }
        k_msleep(TEST_HOLD_MS);
    }

    /* ================================================================
     * GROUP 2: RAW WS2812 FRAME (onboard = CYAN)
     *
     * Manually encodes WS2812 pixel data into SPI bytes using
     * the same algorithm as the WS2812 driver, then sends via
     * raw spi_write(). This isolates "is the encoding wrong?"
     * from "is SPI working at all?"
     *
     * One RGB LED = 3 colors * 8 SPI bytes = 24 bytes.
     * GRB order (matching our overlay color-mapping).
     * ================================================================ */

    LOG_INF("════════════════════════════════════");
    LOG_INF("  GROUP 2: RAW WS2812 FRAME (manual encoding)");
    LOG_INF("════════════════════════════════════");

    if (spi3_dev != NULL && device_is_ready(spi3_dev)) {

        /* Test 2.1: 1 LED, all white, standard frames (0x70/0x40) */
        announce(0, 1, 1, 1);  /* cyan */
        LOG_INF("T2.1: Manual WS2812 frame: 1 LED WHITE (GRB=FF,FF,FF)");
        LOG_INF("  one_frame=0x70 zero_frame=0x40 (standard)");
        memset(raw_spi_buf, 0, RAW_SPI_BUF_SIZE);
        encode_ws2812_byte(&raw_spi_buf[0],  0xFF, 0x70, 0x40);  /* G */
        encode_ws2812_byte(&raw_spi_buf[8],  0xFF, 0x70, 0x40);  /* R */
        encode_ws2812_byte(&raw_spi_buf[16], 0xFF, 0x70, 0x40);  /* B */
        raw_spi_send(spi3_dev, SPI_OP_MODE0, raw_spi_buf, 24,
                     "1LED white 0x70/0x40");
        k_msleep(TEST_HOLD_MS);

        /* Test 2.2: Same as 2.1 but 10x rapid sends */
        announce(0, 1, 1, 2);
        LOG_INF("T2.2: Same frame 10x rapid sends (100ms apart)");
        for (int i = 0; i < 10; i++) {
            rc = raw_spi_send(spi3_dev, SPI_OP_MODE0, raw_spi_buf, 24,
                              "1LED white repeat");
            k_msleep(100);
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 2.3: Different one/zero frame values.
         * 0x7C = 01111100 = wider "one" pulse (5/8 of byte period)
         * 0x60 = 01100000 = wider "zero" pulse (2/8 of byte period)
         * Some strips need different timing. */
        announce(0, 1, 1, 3);
        LOG_INF("T2.3: Manual WS2812: 1 LED WHITE, alt frames 0x7C/0x60");
        memset(raw_spi_buf, 0, RAW_SPI_BUF_SIZE);
        encode_ws2812_byte(&raw_spi_buf[0],  0xFF, 0x7C, 0x60);  /* G */
        encode_ws2812_byte(&raw_spi_buf[8],  0xFF, 0x7C, 0x60);  /* R */
        encode_ws2812_byte(&raw_spi_buf[16], 0xFF, 0x7C, 0x60);  /* B */
        for (int i = 0; i < 5; i++) {
            raw_spi_send(spi3_dev, SPI_OP_MODE0, raw_spi_buf, 24,
                         "1LED white 0x7C/0x60");
            k_msleep(100);
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 2.4: Different one/zero frame values — even wider.
         * 0x7E = 01111110 = 6/8 one pulse
         * 0x40 = 01000000 = 1/8 zero pulse
         * This produces very different timing from standard. */
        announce(0, 1, 1, 4);
        LOG_INF("T2.4: Manual WS2812: 1 LED WHITE, alt frames 0x7E/0x40");
        memset(raw_spi_buf, 0, RAW_SPI_BUF_SIZE);
        encode_ws2812_byte(&raw_spi_buf[0],  0xFF, 0x7E, 0x40);  /* G */
        encode_ws2812_byte(&raw_spi_buf[8],  0xFF, 0x7E, 0x40);  /* R */
        encode_ws2812_byte(&raw_spi_buf[16], 0xFF, 0x7E, 0x40);  /* B */
        for (int i = 0; i < 5; i++) {
            raw_spi_send(spi3_dev, SPI_OP_MODE0, raw_spi_buf, 24,
                         "1LED white 0x7E/0x40");
            k_msleep(100);
        }
        k_msleep(TEST_HOLD_MS);

        /* Test 2.5: 1 LED RED only (G=0, R=FF, B=0) — simpler pattern,
         * fewer "one" pulses in the data stream */
        announce(0, 1, 1, 5);
        LOG_INF("T2.5: Manual WS2812: 1 LED RED (GRB=00,FF,00)");
        memset(raw_spi_buf, 0, RAW_SPI_BUF_SIZE);
        encode_ws2812_byte(&raw_spi_buf[0],  0x00, 0x70, 0x40);  /* G=0 */
        encode_ws2812_byte(&raw_spi_buf[8],  0xFF, 0x70, 0x40);  /* R=FF */
        encode_ws2812_byte(&raw_spi_buf[16], 0x00, 0x70, 0x40);  /* B=0 */
        for (int i = 0; i < 5; i++) {
            raw_spi_send(spi3_dev, SPI_OP_MODE0, raw_spi_buf, 24,
                         "1LED red 0x70/0x40");
            k_msleep(100);
        }
        k_msleep(TEST_HOLD_MS);

    } else {
        LOG_ERR("GROUP 2: Skipped — SPI3 device not available");
        k_msleep(2000);
    }

    /* ================================================================
     * GROUP 3: WS2812 DRIVER STRIP TESTS (onboard = GREEN)
     * ================================================================ */

    LOG_INF("════════════════════════════════════");
    LOG_INF("  GROUP 3: WS2812 DRIVER STRIP COLORS");
    LOG_INF("════════════════════════════════════");

    /* Test 3.1: All WHITE */
    announce(0, 1, 0, 1);
    LOG_INF("T3.1: All WHITE (255,255,255) — 13 LEDs");
    strip_fill(255, 255, 255);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 3.2: All RED */
    announce(0, 1, 0, 2);
    LOG_INF("T3.2: All RED (255,0,0)");
    strip_fill(255, 0, 0);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 3.3: All GREEN */
    announce(0, 1, 0, 3);
    LOG_INF("T3.3: All GREEN (0,255,0)");
    strip_fill(0, 255, 0);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 3.4: All BLUE */
    announce(0, 1, 0, 4);
    LOG_INF("T3.4: All BLUE (0,0,255)");
    strip_fill(0, 0, 255);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 3.5: 5x consecutive sends */
    announce(0, 1, 0, 5);
    LOG_INF("T3.5: 5x consecutive sends, all WHITE, 100ms apart");
    strip_fill(255, 255, 255);
    for (int i = 0; i < 5; i++) {
        rc = strip_send();
        LOG_INF("  send %d: rc=%d", i + 1, rc);
        k_msleep(100);
    }
    k_msleep(TEST_HOLD_MS);

    /* Test 3.6: Reset timing test */
    announce(0, 1, 0, 6);
    LOG_INF("T3.6: Send OFF, wait 1s (reset), then WHITE");
    strip_fill(0, 0, 0);
    strip_send();
    k_msleep(1000);
    strip_fill(255, 255, 255);
    strip_send();
    k_msleep(TEST_HOLD_MS);

    /* ================================================================
     * GROUP 4: PATTERNS (onboard = BLUE)
     * ================================================================ */

    LOG_INF("════════════════════════════════════");
    LOG_INF("  GROUP 4: PATTERNS");
    LOG_INF("════════════════════════════════════");

    /* Test 4.1: Walking LED */
    announce(0, 0, 1, 1);
    LOG_INF("T4.1: Walking LED (single white, all 13 positions)");
    for (int i = 0; i < TOTAL_LEDS; i++) {
        strip_clear();
        strip_set(i, 255, 255, 255);
        strip_send();
        k_msleep(400);
    }
    k_msleep(500);

    /* Test 4.2: First LED only */
    announce(0, 0, 1, 2);
    LOG_INF("T4.2: LED[0] only = WHITE");
    strip_clear();
    strip_set(0, 255, 255, 255);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* Test 4.3: R/G/B cycling */
    announce(0, 0, 1, 3);
    LOG_INF("T4.3: Each LED different (R/G/B cycle)");
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

    /* Test 4.4: ALL OFF — inversion check */
    announce(0, 0, 1, 4);
    LOG_INF("T4.4: All OFF (0,0,0) — ** IF LIT = INVERTED SIGNAL **");
    strip_fill(0, 0, 0);
    rc = strip_send();
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* ================================================================
     * GROUP 5: CHAIN LENGTH (onboard = MAGENTA)
     * ================================================================ */

    LOG_INF("════════════════════════════════════");
    LOG_INF("  GROUP 5: CHAIN LENGTH TESTS");
    LOG_INF("════════════════════════════════════");

    announce(1, 0, 1, 1);
    LOG_INF("T5.1: chain=1, LED[0]=WHITE");
    strip_clear();
    strip_set(0, 255, 255, 255);
    rc = strip_send_n(1);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    announce(1, 0, 1, 2);
    LOG_INF("T5.2: chain=2, LED[0..1]=WHITE");
    strip_clear();
    strip_set(0, 255, 255, 255);
    strip_set(1, 255, 255, 255);
    rc = strip_send_n(2);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    announce(1, 0, 1, 3);
    LOG_INF("T5.3: chain=3, all WHITE");
    strip_fill(255, 255, 255);
    rc = strip_send_n(3);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    announce(1, 0, 1, 4);
    LOG_INF("T5.4: chain=8, all WHITE");
    strip_fill(255, 255, 255);
    rc = strip_send_n(8);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    announce(1, 0, 1, 5);
    LOG_INF("T5.5: chain=13 (full), all WHITE");
    strip_fill(255, 255, 255);
    rc = strip_send_n(13);
    LOG_INF("  rc=%d", rc);
    k_msleep(TEST_HOLD_MS);

    /* ================================================================
     * GROUP 6: PIN IDENTITY TEST (onboard = RED)
     *
     * Runs LAST to avoid potentially interfering with SPI3 MOSI pin.
     * Toggles D8, D9, D10 as raw GPIO.
     * Blink count: 1=D8, 2=D9, 3=D10.
     *
     * On nRF52840, gpio_pin_configure() will succeed regardless of
     * whether SPI3 has claimed the pin (GPIO and peripheral pin
     * control are independent). The SPIM peripheral overrides GPIO
     * when active, so this shouldn't break SPI in the next cycle.
     * ================================================================ */

    LOG_INF("════════════════════════════════════");
    LOG_INF("  GROUP 6: PIN IDENTITY (probe D8/D9/D10 with multimeter)");
    LOG_INF("════════════════════════════════════");

    for (int i = 0; i < SPI_PIN_COUNT; i++) {
        struct test_pin *p = &spi_pins[i];
        announce(1, 0, 0, p->announce_count);

        LOG_INF("T6.%d: GPIO toggle %s — probe pad now!", i + 1, p->label);
        rc = test_gpio_toggle(*(p->port), p->pin, PIN_TOGGLE_MS, p->label);

        if (rc == 0) {
            LOG_INF("  T6.%d: OK — pin toggled as GPIO", i + 1);
        } else {
            LOG_WRN("  T6.%d: FAILED rc=%d", i + 1, rc);
        }
        onboard_off();
        k_msleep(500);
    }

    LOG_INF("────────────────────────────────────");
    LOG_INF("PIN IDENTITY: 1 blink=D8, 2 blinks=D9, 3 blinks=D10");
    LOG_INF("  Probe each pad. Whichever toggles is that pin.");
    LOG_INF("────────────────────────────────────");
    k_msleep(2000);

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
    init_pin_table();

    onboard_set(1, 1, 0);
    LOG_INF("╔═════════════════════════════════════════════╗");
    LOG_INF("║  LED DIAGNOSTIC — SPI + PIN IDENTITY v2    ║");
    LOG_INF("║  Groups: RawSPI / RawWS2812 / Driver /     ║");
    LOG_INF("║          Patterns / ChainLen / PinID        ║");
    LOG_INF("║  (SPI tests first, GPIO pin test last)     ║");
    LOG_INF("╚═════════════════════════════════════════════╝");
    k_msleep(1500);
    onboard_off();

    /* Get the strip device */
    strip = DEVICE_DT_GET(DT_NODELABEL(led_strip));
    LOG_INF("Strip device: name='%s', ready=%s",
            strip ? strip->name : "(null)",
            device_is_ready(strip) ? "YES" : "NO");

    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device NOT READY — looping red blink forever");
        LOG_ERR("Check: CONFIG_SPI=y, CONFIG_LED_STRIP=y, CONFIG_WS2812_STRIP=y");
        while (1) {
            onboard_blink(1, 0, 0, 3, 150);
            k_msleep(500);
        }
        return -ENODEV;
    }

    /* Log DT status */
#if DT_NODE_EXISTS(DT_NODELABEL(spi3))
    {
        const struct device *s = DEVICE_DT_GET(DT_NODELABEL(spi3));
        LOG_INF("SPI3: name='%s', ready=%s",
                s->name, device_is_ready(s) ? "YES" : "NO");
    }
#else
    LOG_ERR("DT node 'spi3': MISSING");
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(spi2))
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi2), okay)
    LOG_WRN("SPI2 is ENABLED — may conflict with SPI3 on shared pins!");
#else
    LOG_INF("SPI2: disabled (good — not conflicting)");
#endif
#endif

    LOG_INF("led_strip DT node: EXISTS=%d",
            DT_NODE_EXISTS(DT_NODELABEL(led_strip)));

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
