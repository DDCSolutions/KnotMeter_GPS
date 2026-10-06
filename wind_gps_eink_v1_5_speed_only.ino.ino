/*
 * GPS Speed Display
 * =================
 * Hardware : Generic ESP32 DevKit (30-pin)
 * Display  : Waveshare 4.2" V2 B&W e-ink (GDEY042T81) 400x300px SPI
 * GPS      : REYAX RYS352A V1.0  — Serial2 @ 115200 baud
 * Power    : 12V switched supply
 *
 * Pin assignments (conflict-free):
 * ─────────────────────────────────
 *  E-ink SCK   → GPIO 18   (HW SPI clock)
 *  E-ink MOSI  → GPIO 23   (HW SPI data)
 *  E-ink CS    → GPIO  5
 *  E-ink DC    → GPIO  2   (moved from original 17 to free GPS UART)
 *  E-ink RST   → GPIO 15   (moved from original 16 to free GPS UART)
 *  E-ink BUSY  → GPIO  4
 *  GPS TX      → GPIO 16   (Serial2 RX on ESP32)
 *  GPS RX      → GPIO 17   (Serial2 TX on ESP32)
 *
 * Libraries required (install via Arduino Library Manager):
 *   GxEPD2          — e-ink display driver
 *   Adafruit GFX    — graphics primitives
 *   Adafruit BusIO  — I2C/SPI abstraction (GxEPD2 dependency)
 *   TinyGPS++       — GPS NMEA parser
 *
 * Display layout (400 x 300 px):
 *   Top    : speed trend arrow, speed (bold integer + regular decimals), "knts"
 *   Bottom : trip odometer (nm), average speed and max speed (kts)
 *
 * Trip stats reset on power-up. Only samples at or above
 * MOVING_THRESHOLD_KTS count toward odometer/average, to ignore GPS drift.
 *
 * Refresh strategy:
 *   Full refresh   : on boot, then every FULL_REFRESH_INTERVAL cycles
 *   Partial refresh: single window inset 16px from display edges
 *
 * GPS speed freshness:
 *   TinyGPS++'s isValid() reports whether a sentence has EVER parsed
 *   successfully — it does NOT reset if new sentences stop arriving, so a
 *   stalled/corrupted NMEA stream reads as "still valid" while quietly
 *   showing the last good value forever. Freshness is tracked separately via
 *   gps.speed.age() against GPS_STALE_TIMEOUT_MS; the GPS UART also gets a
 *   larger RX ring buffer so bytes aren't dropped while the loop is blocked
 *   inside an e-ink refresh. See updateGPS() for detail.
 *
 * Version history:
 *   1.0  Initial merge of OLED receiver + e-ink display sketches
 *   1.1  Fixed refresh: single left-panel partial window, static gauge graphic
 *   1.2  Fixed perimeter pulsing, pointer partial window, black/white clear
 *   1.3  Fixed white-box corruption: right gauge panel now fully redrawn each
 *        partial cycle. Left panel inset increased to 10px. Removed pointer-only
 *        partial window approach which is fundamentally incompatible with e-ink
 *        partial refresh fillScreen behaviour.
 *   1.4  Removed shutdown-detection interrupt and shutdown clear sequence
 *        (hardware no longer present). Removed minor (5°) wind gauge ticks,
 *        major ticks/labels/arc/pointer unchanged. Fixed GPS speed value
 *        freezing: enlarged GPS UART RX buffer and added age-based staleness
 *        check so a stalled fix now shows "--.-" instead of a stuck value;
 *        display refresh cadence and partial/full refresh logic untouched.
 *   1.5  Speed-only: removed wind angle, ESP-NOW receiver and wind gauge.
 *   1.6  Adjusted label Y position for better alignment with speed value.
 *   1.7  New layout: trend arrow, split-weight speed, trip odo/avg/max row.
 */

// ── Core ───────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <SPI.h>

// ── GPS ────────────────────────────────────────────────────────────────────
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ── E-ink display ──────────────────────────────────────────────────────────
#include <GxEPD2_BW.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Fonts/FreeSans18pt7b.h>

// ══════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ══════════════════════════════════════════════════════════════════════════

// E-ink SPI
#define EPD_SCK     18
#define EPD_MOSI    23
#define EPD_CS       5
#define EPD_DC       2    // moved — was 17 in original e-ink sketch
#define EPD_RST     15    // moved — was 16 in original e-ink sketch
#define EPD_BUSY     4

// GPS Serial2
#define GPS_RX      16    // ESP32 Serial2 RX  ← GPS TX wire
#define GPS_TX      17    // ESP32 Serial2 TX  → GPS RX wire
#define GPS_BAUD    115200

// GPS UART RX ring buffer — enlarged from the ESP32 HardwareSerial default
// (256 bytes) so incoming NMEA bytes aren't dropped while the loop is
// blocked inside an e-ink partial/full refresh (which can take from a few
// hundred ms up to a couple of seconds). Must be set before gpsSerial.begin().
#define GPS_RX_BUFFER_BYTES  2048

// ══════════════════════════════════════════════════════════════════════════
//  DISPLAY OBJECT
// ══════════════════════════════════════════════════════════════════════════

GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT>
    display(GxEPD2_420_GDEY042T81(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

static const int DISP_W = 400;
static const int DISP_H = 300;

// ══════════════════════════════════════════════════════════════════════════
//  LAYOUT CONSTANTS
// ══════════════════════════════════════════════════════════════════════════

// Main speed readout (right edge of decimals is fixed so the point doesn't jump)
static const int SPEED_RIGHT_X = 290;
static const int SPEED_Y       = 115;
static const int UNIT_X        = 306;
static const int UNIT_Y        = 145;

// Trend arrow
static const int ARROW_CX     = 32;
static const int ARROW_TOP    = 60;
static const int ARROW_BOTTOM = 102;

// Trip stats row — columns are right-aligned to these x positions
static const int STAT_LABEL_Y = 232;
static const int STAT_VALUE_Y = 276;
static const int STAT_COL_RIGHT[3] = { 112, 246, 380 };

// Partial refresh box.
// Inset 16px from all display edges — GDEY042T81 border waveform fires
// reliably within ~14px of the physical edge on this panel revision.
static const int BOX_X = 16;
static const int BOX_Y = 16;
static const int BOX_W = DISP_W - 32;
static const int BOX_H = DISP_H - 32;

// Full refresh cadence — every N cycles do a full panel refresh to clear ghosting.
static const uint32_t FULL_REFRESH_INTERVAL = 60;

// ══════════════════════════════════════════════════════════════════════════
//  LIVE DATA  (updated by GPS parse)
// ══════════════════════════════════════════════════════════════════════════

static float gps_speed_kts = 0.0f;   // knots from GPS
static bool  gps_valid     = false;  // true when GPS has a fresh, valid fix

// Max age of the last successfully-parsed speed sentence before it is
// treated as stale rather than displayed as a live value. GPS speed
// sentences (RMC/VTG) typically arrive at 1Hz, so 3s tolerates a couple of
// missed/corrupted sentences without false-flagging, while still catching a
// genuinely stalled feed quickly.
static const uint32_t GPS_STALE_TIMEOUT_MS = 3000;

HardwareSerial gpsSerial(1);   // Serial2 on GPIO16/17 via UART1
TinyGPSPlus    gps;

// ══════════════════════════════════════════════════════════════════════════
//  TRIP STATS  (since power-up)
// ══════════════════════════════════════════════════════════════════════════

static const float MOVING_THRESHOLD_KTS = 0.5f;
static const float TREND_DEADBAND_KTS   = 0.05f;
static const float METERS_PER_NM        = 1852.0f;

static float    trip_nm        = 0.0f;
static float    trip_max_kts   = 0.0f;
static float    trip_speed_sum = 0.0f;
static uint32_t trip_samples   = 0;
static int      speed_trend    = 0;   // +1 rising, -1 falling, 0 steady

// Must run before anything else reads gps.speed/location, which clears isUpdated().
void updateTripStats() {
    static float  prev_kts = -1.0f;
    static double prev_lat = 0.0, prev_lng = 0.0;
    static bool   have_prev_loc = false;

    if (gps.speed.isUpdated() && gps.speed.isValid()) {
        float kts = gps.speed.knots();
        if (prev_kts >= 0.0f) {
            float delta = kts - prev_kts;
            speed_trend = (delta >  TREND_DEADBAND_KTS) ?  1 :
                          (delta < -TREND_DEADBAND_KTS) ? -1 : 0;
        }
        prev_kts = kts;

        if (kts >= MOVING_THRESHOLD_KTS) {
            trip_speed_sum += kts;
            trip_samples++;
        }
        if (kts > trip_max_kts) trip_max_kts = kts;
    }

    if (gps.location.isUpdated() && gps.location.isValid()) {
        double lat = gps.location.lat();
        double lng = gps.location.lng();
        if (have_prev_loc && gps.speed.knots() >= MOVING_THRESHOLD_KTS) {
            trip_nm += TinyGPSPlus::distanceBetween(prev_lat, prev_lng, lat, lng) / METERS_PER_NM;
        }
        prev_lat = lat;
        prev_lng = lng;
        have_prev_loc = true;
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  GPS
// ══════════════════════════════════════════════════════════════════════════

// updateGPS() diagnosis (periodic "frozen" speed value):
//   1) gpsSerial used the ESP32 HardwareSerial default 256-byte RX buffer.
//      The e-ink refresh (partialRefresh()/fullRefresh()) blocks loop() for
//      anywhere from a few hundred ms to a couple of seconds while it waits
//      on the panel BUSY line. NMEA bursts arriving during that window could
//      overrun the small buffer, dropping bytes and corrupting/losing
//      sentences (checksum failures inside TinyGPS++).
//   2) Worse, gps.speed.isValid() only reports whether a sentence has EVER
//      parsed successfully — it never goes false again just because updates
//      stopped arriving. So once corrupted/missing sentences interrupted the
//      stream, isValid() kept returning true and gps.speed.knots() kept
//      returning the last good value, i.e. the displayed speed appeared to
//      "freeze" without any error being visible.
// Fix: gpsSerial is given a larger RX ring buffer (set in setup(), before
// begin()) so bursts survive a blocked refresh, and freshness is verified
// explicitly via gps.speed.age() rather than trusting isValid() alone. This
// function stays non-blocking and is called from loop() exactly as before,
// so the periodic display update timing is unaffected.
void updateGPS() {
    // Non-blocking: drain whatever bytes arrived since last call
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    updateTripStats();

    if (gps.speed.isValid() && gps.speed.age() <= GPS_STALE_TIMEOUT_MS) {
        gps_speed_kts = gps.speed.knots();
        gps_valid = true;
    } else {
        // Either never had a fix, or the last good sentence is older than
        // GPS_STALE_TIMEOUT_MS — treat as no data rather than show a stale
        // frozen number.
        gps_valid = false;
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  DRAWING
// ══════════════════════════════════════════════════════════════════════════

void printRightAligned(const char* text, int rightX, int baselineY) {
    int16_t bx, by;
    uint16_t tw, th;
    display.getTextBounds(text, 0, 0, &bx, &by, &tw, &th);
    display.setCursor(rightX - (int)tw - bx, baselineY);
    display.print(text);
}

// ── Black border — drawn on every refresh to mask perimeter waveform flash ─
// 3px thick rectangle just inside the display edge. The hardware border
// waveform causes the outermost pixels to flash white briefly during partial
// refresh. A solid black border drawn on every update makes this invisible.
void drawBorder() {
    for (int i = 0; i < 3; i++) {
        display.drawRect(i, i, DISP_W - i*2, DISP_H - i*2, GxEPD_BLACK);
    }
}

// Outlined arrow: dir +1 points up, -1 points down.
void drawTrendArrow(int dir) {
    const int headH = 18, headHalf = 14, shaftHalf = 6, t = 3;
    const int tipY  = (dir > 0) ? ARROW_TOP : ARROW_BOTTOM;
    const int baseY = (dir > 0) ? ARROW_BOTTOM : ARROW_TOP;
    const int headY = tipY + dir * headH;

    int shaftTop = min(headY, baseY);
    int shaftH   = abs(baseY - headY) + 1;
    display.fillTriangle(ARROW_CX, tipY, ARROW_CX - headHalf, headY,
                         ARROW_CX + headHalf, headY, GxEPD_BLACK);
    display.fillRect(ARROW_CX - shaftHalf, shaftTop, shaftHalf * 2 + 1, shaftH, GxEPD_BLACK);

    int innerHeadY = headY - dir * (t - 1);
    int innerBaseY = baseY - dir * t;
    display.fillTriangle(ARROW_CX, tipY + dir * (t + 2),
                         ARROW_CX - headHalf + t + 1, innerHeadY,
                         ARROW_CX + headHalf - t - 1, innerHeadY, GxEPD_WHITE);
    int innerTop = min(innerHeadY - dir, innerBaseY);
    int innerH   = abs(innerBaseY - (innerHeadY - dir)) + 1;
    display.fillRect(ARROW_CX - shaftHalf + t, innerTop,
                     (shaftHalf - t) * 2 + 1, innerH, GxEPD_WHITE);
}

void drawSpeedValue() {
    char buf[16];
    if (gps_valid) {
        snprintf(buf, sizeof(buf), "%.2f", gps_speed_kts);
    } else {
        snprintf(buf, sizeof(buf), "-.--");
    }

    char* dot = strchr(buf, '.');
    const char* frac = dot ? dot : "";
    char intPart[8];
    size_t intLen = dot ? (size_t)(dot - buf) : strlen(buf);
    if (intLen >= sizeof(intPart)) intLen = sizeof(intPart) - 1;
    memcpy(intPart, buf, intLen);
    intPart[intLen] = '\0';

    int16_t bx, by;
    uint16_t tw, th;
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(2);

    display.setFont(&FreeSans24pt7b);
    display.getTextBounds(frac, 0, 0, &bx, &by, &tw, &th);
    int fracX = SPEED_RIGHT_X - (int)tw - bx;
    display.setCursor(fracX, SPEED_Y);
    display.print(frac);

    display.setFont(&FreeSansBold24pt7b);
    int fracInkLeft = fracX + bx;
    display.getTextBounds(intPart, 0, 0, &bx, &by, &tw, &th);
    display.setCursor(fracInkLeft - 6 - (int)tw - bx, SPEED_Y);
    display.print(intPart);

    display.setTextSize(1);
    display.setFont(&FreeSans18pt7b);
    display.setCursor(UNIT_X, UNIT_Y);
    display.print("knts");

    if (gps_valid && speed_trend != 0) {
        drawTrendArrow(speed_trend);
    }
}

void drawTripStats() {
    char odo[12], avg[12], mx[12];
    snprintf(odo, sizeof(odo), trip_nm < 100.0f ? "%.1f" : "%.0f", trip_nm);
    snprintf(avg, sizeof(avg), "%.2f", trip_samples ? trip_speed_sum / trip_samples : 0.0f);
    snprintf(mx,  sizeof(mx),  "%.2f", trip_max_kts);

    const char* labels[3] = { "odo", "avg", "max" };
    const char* values[3] = { odo, avg, mx };

    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        display.setFont(&FreeSans18pt7b);
        printRightAligned(labels[i], STAT_COL_RIGHT[i], STAT_LABEL_Y);
        display.setFont(&FreeSans24pt7b);
        printRightAligned(values[i], STAT_COL_RIGHT[i], STAT_VALUE_Y);
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  REFRESH ROUTINES
// ══════════════════════════════════════════════════════════════════════════

void fullRefresh() {
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        drawBorder();
        drawSpeedValue();
        drawTripStats();
    } while (display.nextPage());
}

// Write Border Waveform Control register directly via SPI.
// GxEPD2 resets this register internally on every partial refresh sequence,
// so it must be re-written after every firstPage/nextPage cycle completes.
// 0x3C = Border Waveform Control register on GDEY042T81 (SSD1683 controller)
// 0x05 = HiZ — border electrode not driven, eliminates perimeter flashing.
void suppressBorder() {
    digitalWrite(EPD_DC, LOW);
    digitalWrite(EPD_CS, LOW);
    SPI.transfer(0x3C);
    digitalWrite(EPD_DC, HIGH);
    SPI.transfer(0x05);
    digitalWrite(EPD_CS, HIGH);
}

void partialRefresh() {
    suppressBorder();
    display.setPartialWindow(BOX_X, BOX_Y, BOX_W, BOX_H);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        drawBorder();
        drawSpeedValue();
        drawTripStats();
    } while (display.nextPage());
    suppressBorder();
}

// ══════════════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println("GPS speed e-ink display — starting up");

    // GPS Serial2 — RX buffer enlarged before begin() so bursts arriving
    // while the loop is blocked in an e-ink refresh aren't dropped (see
    // updateGPS() for the full explanation of the freeze this fixes).
    gpsSerial.setRxBufferSize(GPS_RX_BUFFER_BYTES);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("GPS serial started");

    // E-ink display
    SPI.begin(EPD_SCK, /*MISO*/ -1, EPD_MOSI, EPD_CS);
    display.init(115200, true, 2, false);
    display.setRotation(0);   // landscape 400 x 300

    Serial.println("Display init — drawing first frame");
    fullRefresh();

    // Suppress border waveform — write after fullRefresh so display is idle
    // and SPI bus is free. Register 0x3C = 0x05 (HiZ) on GDEY042T81.
    digitalWrite(EPD_DC,  LOW);
    digitalWrite(EPD_CS,  LOW);
    SPI.transfer(0x3C);
    digitalWrite(EPD_DC,  HIGH);
    SPI.transfer(0x05);
    digitalWrite(EPD_CS,  HIGH);

    Serial.println("Setup complete");
}

// ══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════════════════════════

static uint32_t cycleCount = 0;

void loop() {
    // ── 1. Drain GPS UART and update speed/fix status
    updateGPS();

    // ── 2. Refresh display
    cycleCount++;
    if (cycleCount % FULL_REFRESH_INTERVAL == 0) {
        fullRefresh();
    } else {
        partialRefresh();
    }

    // ── 3. Drain GPS UART again immediately after the refresh.
    // The refresh call above blocks for anywhere from ~300ms (partial) to a
    // couple of seconds (full), during which NMEA bytes queue up in the
    // enlarged RX buffer (see updateGPS()). Draining here — rather than
    // waiting for the top of the next loop pass — keeps the buffer from
    // building up across consecutive cycles and gets fresh speed data
    // parsed as soon as possible. This is a cheap, non-blocking call and
    // does not change the display refresh cadence below.
    updateGPS();

    // ── 4. Pace the loop
    // The e-ink partial refresh itself takes ~300ms so effective rate is
    // roughly 1.2 updates/sec.
    delay(500);
}
