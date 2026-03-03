#!/bin/bash

# Configuration
DOCKER_IMAGE="zmk-ardux:latest"
BUILD_DIR="build/left"
UF2_FILE="$BUILD_DIR/zephyr/zmk.uf2"
MOUNT_NAMES=("XIAO-SENSE" "XIAO-BLE" "XIAO-NRF52")

echo "🚀 Starting Ardux XIAO Build..."

# 1. Run the Docker Build
docker run --rm -v ./:/workspaces/zmk-ardux -w /workspaces/zmk-ardux "$DOCKER_IMAGE" /bin/bash -c "export ZEPHYR_BASE=/workspaces/zmk-ardux/zephyr && export CMAKE_PREFIX_PATH=/workspaces/zmk-ardux/zephyr/share/zephyr-package/cmake && west build -p -b seeeduino_xiao_ble -d build/left zmk/app -- -DSHIELD=ardux_xiao_left -DZMK_CONFIG=/workspaces/zmk-ardux/config"

if [ $? -ne 0 ]; then
  echo "❌ Build failed. Check the logs above."
  exit 1
fi

echo "✅ Build successful!"
echo "------------------------------------------------"

# 2. Attempt Auto-Reset into Bootloader (1200bps touch)
PORT=$(ls /dev/tty.usbmodem* 2>/dev/null | head -n 1)
if [ -n "$PORT" ]; then
  echo "🔌 Found XIAO on $PORT. Attempting auto-reset into bootloader..."
  # 1200bps touch trick
  stty -f "$PORT" 1200
  sleep 2
else
  echo "⚠️  XIAO serial port not found. Please manual double-tap reset."
fi

echo "⏳ Waiting for XIAO drive to mount..."

# 3. Wait for Mount
TARGET_VOL=""
while [ -z "$TARGET_VOL" ]; do
  for name in "${MOUNT_NAMES[@]}"; do
    if [ -d "/Volumes/$name" ]; then
      TARGET_VOL="/Volumes/$name"
      break
    fi
  done
  sleep 1
done

echo "📂 Found mount point: $TARGET_VOL"

# 4. Copy the File
echo "💾 Copying firmware..."
cp "$UF2_FILE" "$TARGET_VOL/"

if [ $? -eq 0 ]; then
  echo "🎉 Deployment complete! The XIAO will reboot now."
else
  echo "❌ Copy failed. Check permissions or connection."
  exit 1
fi
