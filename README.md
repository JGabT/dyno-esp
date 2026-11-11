# Dyno ESP32

Arduino Nano ESP32 dyno dashboard for an **E18-D80NK-N** infrared sensor.

The ESP32 creates an open WiFi access point. Connect to it and open
`http://192.168.4.1` to see live values and 5 Hz charts.

## Features

- RPM from the IR sensor pulse interval, with microsecond timestamps
- Non-blocking ISR pulse capture, 2 ms debounce and pulse-dropout filter
- Moving-average and low-pass filtering of angular velocity before alpha
- Sensor pulse counter and revolution counter
- Angular acceleration (rad/s²) from a short least-squares fit
- Estimated torque and mechanical power (inertial dyno, no load cell)
- Coast-down friction calibration: averages −I × α while the roller coasts,
  then adds that loss back to the torque estimate
- Light dashboard UI
- Live charts at 5 Hz: RPM, torque, power, angular acceleration
- Motor performance characteristics chart: **torque (N·m) on the X axis**,
  with speed (rpm) on the left Y axis and current (A), mechanical power (W)
  and efficiency (%) on three separate right Y axes
- Adjustable dashboard inputs for roller mass, roller radius, pulses per
  revolution and moving-average window
- Start run / Stop run controls and a Format storage button in the dashboard
- Motor efficiency estimate: mechanical roller power ÷ BMS battery power
- CSV logging of every 5 Hz sample to on-board flash
  - one file per run (`/run_0001.csv`, `/run_0002.csv`, …)
  - browse and download runs from the dashboard at `/logs`
- Daly smart BMS telemetry over BLE:
  - pack voltage, current, state of charge, battery power
  - cell max/min voltages and cell spread
  - temperatures, remaining capacity, cycles, charge/discharge MOS state
- 5 Hz JSON telemetry stream on the USB serial port
- Settings saved in ESP32 flash:
  - roller mass (kg), default 43.0
  - roller radius (m), default 0.15
  - effective inertia computed as I = ½ m r² (default 0.48375 kg·m²)
  - friction torque (N·m), set manually or by coast-down calibration
  - pulses per revolution
  - moving-average window size (samples)
- Open WiFi AP, no password

## Files

- `dyno-esp.ino` — complete sketch
- `README.md` — this file
- `flash-linux.sh` — USB recovery/full-flash helper for Linux
- `firmware/` — prebuilt Nano ESP32 bootloader, recovery, partition and app images

`firmware/dyno-esp.bin` is rebuilt from the current sketch. If you change
the sketch, rebuild it with **Sketch → Export Compiled Binary** and replace
that file (or use the normal Arduino upload).

## Wiring

The E18-D80NK-N is a 5 V NPN normally-open sensor. Its black output can
be near 5 V, but the Nano ESP32 GPIO is **3.3 V only**. Use a divider.

| E18-D80NK-N wire | Connect to |
|---|---|
| Brown | Nano ESP32 **5V** |
| Blue | Nano ESP32 **GND** |
| Black (signal) | 10 kΩ resistor in series to **D2**, and 20 kΩ from D2 to **GND** |

```
E18 black ----[ 10k ]----+---- D2
                         |
                       [20k]
                         |
                        GND
```

That gives about 3.3 V at D2 when the sensor output is 5 V.

Most E18 modules have an internal pull-up on the output. If yours is a bare
open-collector NPN output, add a 10 kΩ pull-up from the black signal wire to
**5 V** as well; otherwise the signal may not reach a reliable high level.
A 1 kΩ / 2 kΩ divider can be used instead of 10 kΩ / 20 kΩ for a
lower-impedance, more noise-resistant signal.

The sensor output is pulled **LOW** when it sees the reflective target.
The sketch uses `INPUT_PULLUP` and triggers on the falling edge.

If your sensor has different behaviour, change `SENSOR_EDGE` in the sketch
to `RISING`, or use the opposite type of target.

Power the sensor from the Nano's 5 V pin only if the board is powered from
USB or another 5 V source that can supply the sensor.

Some E18 variants are rated for 6–36 V instead of 5 V. Check the label on
your sensor. If yours needs a higher supply, power it from a separate supply
and connect the grounds together; keep the signal divider on D2.

## Target / pulses per revolution

Put one reflective marker on the rotating shaft or roller. One marker means:

- 1 pulse per revolution (`Pulses per revolution = 1`)

If you use 4 equally spaced reflective markers, set it to 4. The sketch
uses pulse timing only, so any number from 1 to 1000 works, but more
pulses give better low-RPM acceleration data. Keep the pulses evenly spaced.

Use a target that passes the sensor cleanly. Avoid a target that stays in
the sensor beam too long at high RPM; the sketch debounces pulses shorter
than 2 ms, which allows up to 500 pulses per second.

## Flash with Arduino IDE

1. Install the Arduino IDE and the **Arduino ESP32 Boards** core (or the
   Espressif ESP32 core).
2. Select **Arduino Nano ESP32** as the board.
3. Select the correct USB port.
4. Open `dyno-esp.ino` and upload.
5. Open Serial Monitor at **115200 baud** if you want the startup IP and
   settings.

The sketch uses these libraries:

- `WiFi`, `WebServer`, `Preferences`, `LittleFS`, `FFat`, `math` — bundled
  with the ESP32 Arduino core
- **NimBLE-Arduino** (tested with 2.5.2) — install it with the Library
  Manager if it is not present. It is used for the Daly BMS BLE connection.

### Upload error: "No DFU capable USB device available"

The Nano ESP32 uploads through its Arduino bootloader using DFU. If the
board is running a sketch, it may not expose the DFU interface automatically.
Put it into bootloader mode:

1. Press the **RESET** button once.
2. Wait for the RGB LED to start flashing.
3. Press **RESET** again.
4. A green LED should now pulse slowly. The board is in bootloader mode.
5. Select **Arduino Nano ESP32** and the correct port, then upload.

Do not use `lsusb` or `dfu-util -l` alone to check for bootloader mode: the
normal sketch also exposes a DFU runtime interface, so `dfu-util` may find
the board and then fail with `LIBUSB_ERROR_PIPE` while the sketch is running.
The reliable sign is the **green LED pulsing slowly** after the second reset.

If only `sudo dfu-util -l` works, it is a Linux USB-permission problem.
Add a udev rule and reconnect the board:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0070", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-arduino-nano-esp32.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Also check:

- Use a USB-C **data** cable, not a charge-only cable.
- Plug directly into the computer, not through an unpowered hub.
- Disconnect the sensor while testing, so a wiring short cannot disturb USB.
- The reset button must be pressed twice within roughly one second. The
  second press is during the RGB fade at boot.
- If the board appears as an Espressif ROM device instead, use Arduino's
  bootloader recovery procedure below.

### Last resort: restore the Nano ESP32 bootloader

This erases the flash. It is useful when uploads keep failing or the board
is misidentified. The BOOT1 and GND pins are next to each other on the
analog header (BOOT1 is pin 13, GND is pin 14).

1. Unplug the sensor and any other wiring.
2. Connect **BOOT1** to **GND** with a jumper wire. The RGB LED turns green.
3. Keep the jumper connected and press **RST**.
4. Remove the jumper. The RGB LED should stay purple.
5. In Arduino IDE:
   - Tools → Port: select the detected ESP32 port.
   - Tools → Board: **Arduino Nano ESP32**.
   - Tools → Programmer: **Esptool** (it must be checked).
   - Tools → **Burn Bootloader**.
6. Wait for `Hard resetting via RTS pin...`, then press **RST**.
7. Use **Sketch → Upload Using Programmer**, or the normal Upload button.

This wipes saved settings, so the roller returns to the 43 kg / 0.15 m
defaults (I = 0.48375 kg·m²) and the moving-average window returns to 8.

### If Arduino IDE esptool fails with BrokenPipeError

The Nano ESP32 uses native USB, and Arduino IDE's esptool command tries to
toggle DTR/RTS to reset it. That can make `/dev/ttyACM0` disappear and give:

```text
BrokenPipeError: [Errno 32] Broken pipe
```

A prebuilt full-recovery image and Linux script are included in this project.
Put the board in ROM download mode first (BOOT1 to GND, press RST, remove
jumper, RGB LED purple), then run:

```bash
cd /path/to/dyno-esp
./flash-linux.sh /dev/ttyACM0
```

Use whichever `/dev/ttyACM*` port appears. The script uses esptool with
`--before no_reset`, erases the flash, and writes the bootloader, partition
table, recovery firmware and dyno application. It avoids the DTR/RTS reset
that causes the BrokenPipeError.

### Flashing through a 3.3 V USB-to-TTL adapter

If the board's USB port is unreliable, you can bypass it and flash through
the ESP32-S3 UART. You need a **3.3 V** USB-to-TTL adapter (CP2102, CH340,
FTDI, etc.). Do not use a 5 V logic adapter directly on D0/D1.

Wiring:

| USB-TTL adapter | Nano ESP32 |
|---|---|
| GND | GND |
| TX | D0 (RX) |
| RX | D1 (TX) |

Do **not** connect the adapter's power pin unless you know it can supply
the board safely; power the Nano through its USB or a proper 5 V supply.

Then:

1. Connect **BOOT1 / B1** (analog header pin 13) to **GND** (pin 14).
2. Press **RST**.
3. Remove the BOOT1–GND jumper. The RGB LED should be purple.
4. Find the adapter port, usually `/dev/ttyUSB0`.
5. Run:

```bash
cd /path/to/dyno-esp
./flash-linux.sh /dev/ttyUSB0
```

This uses `--before no_reset`, so esptool does not need to toggle the
board's USB/DTR lines. A dedicated 3.3 V USB-TTL adapter is much more
reliable than trying to use a 5 V Arduino board as a serial bridge.

If esptool reports a permission error for the ROM bootloader, add a rule
for the Espressif USB ID too, then unplug/replug:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-esp32-rom.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Use

1. On a phone or computer, join the open WiFi network **Dyno-ESP32**.
2. Open **http://192.168.4.1**.
3. The dashboard updates at 5 Hz.
4. Set the roller mass, radius, PPR and moving-average window under
   **Calibration & settings**, then press **Save settings**.
5. Run the coast-down calibration once with the motor unpowered, or type a
   known friction torque.
6. Press **Start run**, then load the motor. The performance chart plots
   speed, current, power and efficiency against estimated torque.

If your client tries to use mobile data instead of the ESP32 network,
turn mobile data off temporarily or tell the client to stay connected to
the open network.

## Local dashboard preview

`preview/index.html` is a standalone preview of the dashboard. It is built
from the HTML inside `dyno-esp.ino` and replaces the `/data` API with a
simulated dyno and BMS feed.

```bash
python3 preview/build_preview.py --open
```

The preview is for layout and chart checking only. Its BMS voltage is
simulated and clearly labelled; the firmware dashboard reads the pack
voltage from the BMS `0x90` response.

## CSV logging

Every 200 ms (5 Hz) the sketch appends one row to the active run file in
on-board flash. A new file is created at boot and whenever **Start run** (or
**New run**) is pressed. **Stop run** flushes the buffer and stops logging.
Rows are buffered for 2 s and then flushed, so a hard power cut can lose at
most the last few samples.

- **Start run** begins a fresh run/file and resets counters and peaks.
- **Stop run** stops logging and marks the run stop time.
- **Format storage** erases all saved runs and formats LittleFS or FFat.
- Open `/logs` (or **All saved runs** on the dashboard) to list files.
- **Download current CSV** streams the active run as an attachment.
- `/download?file=run_0001.csv` downloads a specific run.
- The filesystem is chosen automatically:
  - If the **With SPIFFS partition (advanced)** scheme is selected in the
    Arduino IDE, LittleFS is used.
  - Otherwise the Nano ESP32 default FAT partition is used (FFat). No IDE
    setting change is needed.
  - The dashboard shows `CSV logging unavailable` if neither mounts.
- The first boot may take a few seconds while the filesystem is formatted.
- Download runs after the pull is finished; transferring a file pauses the
  main loop for the duration of the download.
- When flash usage passes 85 %, the oldest run is deleted when a new run
  starts.

The CSV columns are: record number, run time, uptime, RPM, angular velocity,
angular acceleration, torque, power, efficiency and its validity flag, sensor
state, pulse count, revolutions, peaks, the settings in use (inertia, friction,
PPR, roller mass, roller radius, moving-average window), the BMS age in ms at
the sample instant, and all BMS fields. `BMS_Age_ms` shows how old the BMS
`0x90` packet was when the row was written, so pulse and BLE data can be
aligned after a run.

## Daly BMS over BLE

The sketch connects to the BMS whose MAC address is set in
`BMS_MAC_ADDRESS` (default `d2:1a:01:01:c9:a7`). No extra wiring is
needed beyond powering the BMS.

- Service `fff0`, notify characteristic `fff1`, write characteristic `fff2`.
- The sketch scans for the MAC at boot and reconnects automatically.
- Queries are sent one at a time every 120 ms: pack data (`0x90`), cell
  max/min (`0x91`), temperatures (`0x92`), MOS/remaining capacity (`0x93`)
  and cycles/cell count (`0x94`), interleaved with repeated `0x90` requests
  so voltage, current and SOC stay fresh.
- Battery values appear in their own dashboard section and in every CSV row.
- Pack voltage, current, SOC and the other BMS fields are shown only while a
  fresh `0x90` response has arrived in the last 3 seconds. Stale or
  disconnected values are hidden instead of being displayed as a static
  number.
- If the connection drops, it rescans every 6 s while the dyno is stopped.
  Reconnects are postponed while RPM > 0 so a connection attempt cannot
  stall pulse processing.
- BLE and the WiFi access point share the 2.4 GHz radio. The 5 Hz dashboard
  works fine, but a CSV download may be slower while the BMS is connected.

To use a different BMS, change `BMS_MAC_ADDRESS` in the sketch. The UUIDs
also need changing if the BMS is not a Daly-compatible model.

## Roller inertia

The dashboard computes the roller inertia automatically from the mass and
radius inputs. The defaults are a **43 kg solid cylinder with a 0.15 m radius**:

```text
I = 1/2 × m × r²
I = 0.5 × 43 × 0.15²
I = 0.48375 kg·m²
```

The formula assumes a uniform solid disk/roller. If the roller is a hollow
ring instead, change the calculation in the sketch to `I = m × r²`; if there
are extra rotating parts, use a larger effective mass or add their inertia.

The inertia is recalculated whenever you save new roller mass/radius values.
It is also shown live under the settings form.

## Torque and power estimate

With only an IR speed sensor, torque cannot be measured directly. This is
an **inertial dyno** calculation:

```text
ω = angular velocity                         (rad/s)
α = dω/dt                                    (rad/s²)
τ_mech = I × α                               (N·m)
τ_loss = bearing/friction torque             (N·m)
τ_total = τ_mech + τ_loss                    (N·m)
P_mech = τ_total × ω                         (W)
```

`I` is the effective moment of inertia of the roller and all rotating parts
that accelerate with it. `ω` is angular velocity in rad/s. During acceleration
`τ_mech` is positive; during coasting it is negative.

This method gives useful numbers during acceleration sweeps. It does **not**
measure steady-state torque: at constant speed, α is zero, so the estimated
torque is only the friction value (or zero when friction is 0). A load cell or
brake is required for true steady-state torque.

### Coast-down friction calibration

1. Open **Calibration & settings** on the dashboard.
2. Make sure the motor is unpowered and the roller can spin freely.
3. Click **Coast-down friction calibration**.
4. Spin the roller up (by hand or with a short powered pulse), then release it.
5. The firmware samples for up to 15 s while the roller is slowing down and
   the BMS current is near zero. It averages `τ_loss = −I × α` and saves the
   result as the friction torque.

The calibration needs at least 20 coasting samples between about 95 and
4750 rpm. If it reports “not enough samples”, spin the roller faster or check
that it is really unpowered. You can also type a known friction torque into
the friction field and save it manually.

### Motor efficiency

The dashboard estimates motor efficiency as mechanical output power divided
by electrical input power:

```text
efficiency = P_mech / (pack voltage × battery current) × 100 %
```

- Mechanical power is the inertial-dyno estimate above.
- Electrical power comes from the Daly BMS (`0x90` voltage and current).
- The value is only calculated while the dyno is accelerating, battery
  current is positive (discharge), and the BMS voltage is fresh.
- It is an estimate, not a calibrated motor efficiency measurement. Battery
  current lags the dyno sample slightly because the BMS is polled over BLE;
  the CSV `BMS_Age_ms` column lets you see that lag.

## Measurement details

- `MIN_PULSE_GAP_US = 2000` debounces the sensor (up to 500 pulses/s).
- RPM is computed from the time between consecutive pulses using `micros()`.
- Raw omega samples are smoothed by:
  - a moving average over the adjustable window (default 8 samples), then
  - a first-order low-pass with a 50 ms time constant.
- Angular acceleration is a least-squares slope over the last 500 ms of the
  filtered omega samples. If fewer than three samples are available it
  reports zero. Alpha is then smoothed with a 0.35 EMA.
- If no pulse arrives for about three pulse intervals (minimum 0.5 s), RPM and
  torque fall to zero. A gap longer than 3 s resets the speed history.
- Internal updates run at 50 Hz; the browser polls, the CSV logger records and
  the serial stream emits at 5 Hz.

## Serial telemetry

The USB serial port (115200 baud) emits one JSON line per 5 Hz sample. It
contains the pulse-derived speed/torque data and the matching BMS voltage and
current:

```json
{"t":123456,"rpm":1420.5,"omega":148.75,"alpha":32.10,
 "torque":0.321,"pMech":47.7,"v":48.12,"i":12.40,
 "pElec":596.7,"eff":8.0,"effOk":1,
 "friction":0.0120,"bmsAge":180,"calib":0}
```

`t` is `millis()`, `bmsAge` is how old the most recent BMS `0x90` packet was,
and `pMech`/`pElec` are mechanical and electrical power. The dashboard uses the
same values from `/data`.

## Web/API endpoints

| Method | Path | Purpose |
|---|---|---|
| GET | `/` | dashboard |
| GET | `/data` | JSON telemetry, BMS data and log status |
| GET | `/logs` | HTML list of saved CSV runs |
| GET | `/download?active=1` | download the active run as CSV |
| GET | `/download?file=run_0001.csv` | download a specific run |
| POST | `/config` | save settings (`mass`, `radius`, `friction`, `ppr`, `avgWindow`) |
| POST | `/calibrate` | start a coast-down friction calibration, or cancel if one is running |
| POST | `/start` | start a new CSV run and reset counters/peaks |
| POST | `/stop` | stop CSV logging and freeze the run stop time |
| POST | `/format` | erase all runs and format the log filesystem |
| POST | `/reset` | alias for `/start` |

Example `/data` response:

```json
{
  "millis": 123456,
  "rpm": 1420.5,
  "omega": 148.75,
  "alpha": 32.10,
  "torque": 0.321,
  "power": 47.7,
  "pMech": 47.7,
  "pElec": 596.7,
  "eff": 8.0,
  "effOk": 1,
  "pulses": 1234,
  "revolutions": 1234.0,
  "peakRpm": 2110.0,
  "peakTorque": 0.540,
  "peakPower": 89.2,
  "mass": 43.000,
  "radius": 0.1500,
  "inertia": 0.48375,
  "friction": 0.0120,
  "ppr": 1,
  "avgWindow": 8,
  "sensor": 0,
  "uptime": 321,
  "runMs": 12000,
  "runActive": 1,
  "runStopMs": 0,
  "calibActive": 0,
  "calibState": 0,
  "calibElapsedMs": 0,
  "calibSamples": 0,
  "calibTorque": 0.0000,
  "log": "/run_0003.csv",
  "logged": 642,
  "logOk": 1,
  "bleReady": 1,
  "bms": {
    "connected": 1,
    "fresh": 1,
    "ageMs": 180,
    "ms": 123276,
    "voltage": 48.12,
    "current": 12.4,
    "soc": 87.5,
    "maxCell": 4012,
    "maxCellNo": 5,
    "minCell": 3998,
    "minCellNo": 7,
    "maxTemp": 32.0,
    "minTemp": 30.0,
    "remCapMah": 24500,
    "cycles": 42,
    "chargeMos": 0,
    "dischargeMos": 1
  }
}
```

## Performance and reliability notes

- The dashboard is still served directly by the ESP32; no host PC or external
  service is required beyond a WiFi client.
- `/data` is formatted into a static BSS buffer, so a 5 Hz poll does not
  allocate a 2 KB payload on the 8 KB loop-task stack.
- The browser polls at 5 Hz while visible and backs off to 1 Hz when the tab
  is hidden, so the ESP32 is not serving a screen nobody is looking at.
- Canvas draws are batched with `requestAnimationFrame`; the performance
  chart also stops redrawing until a new torque sample arrives.
- The performance chart keeps 900 samples (3 minutes at 5 Hz), which is far
  more than a normal dyno pull needs.
- The HTML lives in flash (`PROGMEM`) and is streamed with `send_P()`; it is
  not copied into RAM for each page load.
- Firmware currently uses about 34 % of the 3 MB application partition and
  21 % of the ESP32-S3 RAM.
- Torque and efficiency remain estimates. There is no load cell, so
  steady-state torque is not measurable.

## Security note

The access point is intentionally open and has no authentication. That is
what "no password" means for this project. Anyone in WiFi range can view
and change settings. Use it only on an isolated bench, and do not expose it
to an untrusted network.

## Limitations

- Torque is estimated, not measured with a load cell.
- No steady-state torque without a brake/load cell.
- The torque chart only shows accelerating samples (`torque > 0`). Coasting
  samples are excluded so the characteristic curves are not pulled toward the
  zero-torque axis.
- Sensor alignment and reflective target quality affect RPM stability.
- The moving-average window trades noise for lag: larger windows are smoother
  but respond more slowly to fast transients.
- Strong ambient IR or sunlight can affect the E18 sensor.
- The 5 V sensor output must be level-shifted to 3.3 V.
- CSV rows are buffered in RAM and written every 2 s. A power loss can drop
  the last few rows.
- Flash logging wears the flash slowly; the 9 MB partition is far larger
  than normal dyno runs need.
- BMS values are only as good as the Daly protocol data. If the BMS
  disconnects, the last values remain visible in the CSV with
  `BMS_Connected=0`.
- The coast-down calibration assumes the motor is unpowered. If the BMS is
  not connected, the firmware cannot verify that, so make sure the motor is
  really off.
