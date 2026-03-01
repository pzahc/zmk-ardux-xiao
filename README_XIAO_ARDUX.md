# Ardux XIAO Build Guide

This document covers the specific configuration and build steps for the custom **Ardux Left-Handed** keyboard using the **Seeed Studio XIAO nRF52840**.

## Hardware Configuration

*   **Controller:** Seeed Studio XIAO nRF52840 (BLE / Sense)
*   **LED Strip:** 13x WS2812B LEDs
    *   **Data Pin:** **Pad 10 (P1.15)** (MOSI)
    *   **Power:** 5V VBUS
*   **Key Matrix:** 8 keys on Pins D0-D7 (Direct GPIO)

## LED Schema (Custom Driver)

The LEDs are managed by a custom driver in `config/boards/shields/ardux_xiao/led_driver.c`.

1.  **Battery Status (LEDs 0-4):**
    *   The first 5 LEDs show remaining battery life.
    *   **Color:** Dim Green.
2.  **Key Backlights (LEDs 5-12):**
    *   **Idle Color:** Dim Cyan.
    *   **Active Color:** Dim Yellow (triggers when the corresponding key is pressed).

## Signal Integrity Fixes

To support 5V LED strips with the XIAO's 3.3V logic without a level shifter, the following settings are used in `seeeduino_xiao_ble.overlay`:
*   **Pin:** Pad 10 (P1.15) was chosen to avoid conflicts with QSPI/SPI2.
*   **Drive Mode:** `NRF_DRIVE_H0H1` (High Drive) is enabled on the MOSI pin.
*   **Frequency:** 4MHz SPI.
*   **Timing:** `0x70` (One) and `0x60` (Zero) frames to ensure the 3.3V pulses are recognized.

## How to Build

Run the following command from the project root to build the left-hand firmware using the local Docker environment:

```bash
docker run --rm -v ./:/workspaces/zmk-ardux -w /workspaces/zmk-ardux zmk-ardux:latest /bin/bash -c "export ZEPHYR_BASE=/workspaces/zmk-ardux/zephyr && export CMAKE_PREFIX_PATH=/workspaces/zmk-ardux/zephyr/share/zephyr-package/cmake && west build -p -b seeeduino_xiao_ble -d build/left zmk/app -- -DSHIELD=ardux_xiao_left -DZMK_CONFIG=/workspaces/zmk-ardux/config"
```

The output firmware will be generated at: `build/left/zephyr/zmk.uf2`

## How to Flash

1.  **Enter Bootloader:** Double-tap the physical **Reset** button on the XIAO.
2.  **Mount Drive:** A drive named `XIAO-SENSE` (or similar) will appear on your computer.
3.  **Deploy:** Copy the `zmk.uf2` file to that drive. The device will automatically reboot with the new firmware.

## Debugging

USB Serial Logging is enabled. To view live logs:
```bash
# On macOS
screen /dev/tty.usbmodem* 115200
```
*(Exit screen using `Ctrl-A` then `K`)*
