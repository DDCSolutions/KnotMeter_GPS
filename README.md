# KnotMeter_GPS
# GPS Speed Display (ESP32 + 4.2" E-ink)

A boat-oriented GPS speedometer built on a generic ESP32 DevKit and a Waveshare 4.2" black & white e-ink panel. It shows current speed over ground in knots with a rising/falling trend arrow, plus a trip odometer, average speed and max speed since power-up.

## Hardware

| Component | Part | Notes |
|-----------|------|-------|
| MCU | Generic ESP32 DevKit (30-pin) | |
| Display | Waveshare 4.2" V2 B&W e-ink (GDEY042T81, SSD1683) | 400 × 300 px, SPI |
| GPS | REYAX RYS352A V1.0 | NMEA over UART @ 115200 baud |
| Power | 12 V switched supply | |

### Wiring

| Signal | ESP32 GPIO | Notes |
|--------|-----------:|-------|
| E-ink SCK | 18 | HW SPI clock |
| E-ink MOSI | 23 | HW SPI data |
| E-ink CS | 5 | |
| E-ink DC | 2 | moved from 17 to free the GPS UART |
| E-ink RST | 15 | moved from 16 to free the GPS UART |
| E-ink BUSY | 4 | |
| GPS TX | 16 | ESP32 Serial2 RX |
| GPS RX | 17 | ESP32 Serial2 TX |

## Required libraries

Install via the Arduino Library Manager:

- **GxEPD2** — e-ink display driver
- **Adafruit GFX** — graphics primitives
- **Adafruit BusIO** — GxEPD2 dependency
- **TinyGPS++** — NMEA parser

Board package: *ESP32 by Espressif Systems*. Select a generic **ESP32 Dev Module** board.

## Display layout

```
┌───────────────────────────────────────────┐
│  ▲        12.34  knts                      │   ← trend arrow, speed (bold int + decimals), unit
│                                            │
│                                            │
│      odo          avg          max         │   ← labels
│      4.2        5.67         8.91          │   ← values (nm / kts / kts)
└───────────────────────────────────────────┘
```

- **Speed** — integer part in bold, decimals in regular weight; right edge is fixed so the decimal point doesn't jump.
- **Trend arrow** — outlined up/down arrow shown when speed changes by more than `TREND_DEADBAND_KTS` (0.05 kt) between samples. Hidden when steady or when there's no fix.
- **No fix / stale data** — speed shows `-.--`.
- **odo** — trip distance in nautical miles (1 decimal below 100 nm, integer above).
- **avg / max** — average and maximum speed in knots.

## Behaviour

### Trip statistics
Trip stats reset on power-up. Only samples at or above `MOVING_THRESHOLD_KTS` (0.5 kt) contribute to the odometer and average, which filters out GPS drift while stationary.

### Refresh strategy
- **Full refresh** on boot, then every `FULL_REFRESH_INTERVAL` (60) cycles to clear ghosting.
- **Partial refresh** otherwise, using a single window inset 16 px from the panel edges.
- A 3 px black border is drawn on every frame and the SSD1683 Border Waveform Control register (`0x3C`) is set to HiZ (`0x05`) around each partial refresh to suppress perimeter flashing.
- Loop pacing is `delay(500)`; with ~300 ms partial refresh time the effective update rate is roughly 1.2 Hz.

### GPS freshness
TinyGPS++'s `isValid()` only reports that a sentence has *ever* parsed — it never goes false if the stream stalls. The sketch therefore:

1. Enlarges the UART RX ring buffer to 2048 bytes (`GPS_RX_BUFFER_BYTES`) so NMEA bursts aren't dropped while `loop()` is blocked inside an e-ink refresh.
2. Checks `gps.speed.age()` against `GPS_STALE_TIMEOUT_MS` (3000 ms) and shows `-.--` instead of a frozen value when the feed stalls.
3. Drains the UART both before and immediately after each refresh.

## Tunable constants

| Constant | Default | Purpose |
|----------|--------:|---------|
| `GPS_BAUD` | 115200 | GPS UART baud rate |
| `GPS_RX_BUFFER_BYTES` | 2048 | UART RX ring buffer size |
| `GPS_STALE_TIMEOUT_MS` | 3000 | Max age of last speed sentence before showing `-.--` |
| `MOVING_THRESHOLD_KTS` | 0.5 | Minimum speed counted toward odo/avg |
| `TREND_DEADBAND_KTS` | 0.05 | Speed change required to show trend arrow |
| `FULL_REFRESH_INTERVAL` | 60 | Cycles between full refreshes |
| `SPEED_RIGHT_X` / `SPEED_Y` | 290 / 115 | Speed readout anchor |
| `STAT_COL_RIGHT[]` | 112, 246, 380 | Right edges of the three stat columns |

## Serial debug

Open the Serial Monitor at **115200 baud** to see startup messages (`GPS serial started`, `Display init`, `Setup complete`).

## Version history

| Version | Change |
|---------|--------|
| 1.0 | Initial merge of OLED receiver + e-ink display sketches |
| 1.1 | Single left-panel partial window, static gauge graphic |
| 1.2 | Fixed perimeter pulsing, pointer partial window, black/white clear |
| 1.3 | Fixed white-box corruption; right gauge panel redrawn each partial cycle |
| 1.4 | Removed shutdown-detection hardware; fixed GPS speed freeze (RX buffer + staleness check) |
| 1.5 | Speed-only: removed wind angle, ESP-NOW receiver and wind gauge |
| 1.6 | Adjusted unit label alignment |
| 1.7 | New layout: trend arrow, split-weight speed, trip odo/avg/max row |
