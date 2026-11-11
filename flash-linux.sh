#!/usr/bin/env bash
#
# flash-linux.sh
# Full recovery flash for the Arduino Nano ESP32 using the ESP32-S3 ROM
# bootloader. This bypasses the Arduino DFU upload problem by using
# esptool with --before no_reset.
#
# Usage:
#   ./flash-linux.sh                 # auto-detect /dev/ttyACM*
#   ./flash-linux.sh /dev/ttyACM1    # use a specific port
#
# The board MUST be in ROM download mode first:
#   1. Unplug USB.
#   2. Connect BOOT1 (analog header pin 13) to GND (pin 14).
#   3. Plug USB in while the jumper is connected -> RGB LED green.
#   4. Press RST.
#   5. Remove the jumper -> RGB LED should stay purple.
#   6. Run this script.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$ROOT/firmware"
PORT="${1:-}"

ESPTOOL="${ESPTOOL:-$HOME/.arduino15/packages/esp32/tools/esptool_py/4.5.1/esptool.py}"
PYTHON="${PYTHON:-python3}"

if [[ ! -f "$ESPTOOL" ]]; then
    if command -v esptool.py >/dev/null 2>&1; then
        ESPTOOL="$(command -v esptool.py)"
    elif command -v esptool >/dev/null 2>&1; then
        ESPTOOL="$(command -v esptool)"
    else
        echo "ERROR: esptool not found."
        echo "Set ESPTOOL=/path/to/esptool.py and run again."
        exit 1
    fi
fi

if [[ -z "$PORT" ]]; then
    mapfile -t PORTS < <(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
    if [[ ${#PORTS[@]} -eq 1 ]]; then
        PORT="${PORTS[0]}"
    elif [[ ${#PORTS[@]} -eq 0 ]]; then
        echo "ERROR: no /dev/ttyACM* or /dev/ttyUSB* port found."
        echo "Enter ROM download mode, then check 'ls /dev/ttyACM* /dev/ttyUSB*'."
        exit 1
    else
        echo "Please choose a port:"
        printf '  %s\n' "${PORTS[@]}"
        echo
        echo "Then run: $0 /dev/ttyACM0"
        exit 1
    fi
fi

if [[ ! -e "$PORT" ]]; then
    echo "ERROR: $PORT does not exist."
    echo "Run 'ls /dev/ttyACM* /dev/ttyUSB*' after entering ROM download mode."
    exit 1
fi

for f in bootloader.bin partitions.bin boot_app0.bin nora_recovery.bin dyno-esp.bin; do
    if [[ ! -f "$FW/$f" ]]; then
        echo "ERROR: missing $FW/$f"
        exit 1
    fi
done

echo
echo "Nano ESP32 full recovery flash"
echo "Port:    $PORT"
echo "Esptool: $ESPTOOL"
echo
echo "The board must be in ROM download mode:"
echo "  BOOT1 (pin 13) + GND (pin 14) -> plug USB -> RGB green"
echo "  press RST -> remove jumper -> RGB stays purple"
echo
read -r -p "Press Enter when the RGB LED is purple... " _

echo
echo "Checking ROM bootloader connection..."
"$PYTHON" "$ESPTOOL" \
    --chip esp32s3 \
    --port "$PORT" \
    --before no_reset \
    --after no_reset \
    chip_id

echo
echo "Erasing and writing full firmware..."
"$PYTHON" "$ESPTOOL" \
    --chip esp32s3 \
    --port "$PORT" \
    --before no_reset \
    --after no_reset \
    write_flash \
    --erase-all \
    -z \
    --flash_mode dio \
    --flash_freq 80m \
    --flash_size 16MB \
    0x0       "$FW/bootloader.bin" \
    0x8000    "$FW/partitions.bin" \
    0xe000    "$FW/boot_app0.bin" \
    0xF70000  "$FW/nora_recovery.bin" \
    0x10000   "$FW/dyno-esp.bin"

echo
echo "Done. Press RST once."
echo "Then connect to WiFi 'Dyno-ESP32' and open http://192.168.4.1"
