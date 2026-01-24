// LD2410B simple test for ESP32-S3-WROOM-2
// Wiring (ESP32-S3 <-> LD2410B):
//   IO14 (TX) -> Radar RX
//   IO13 (RX) <- Radar TX
//   IO12 (IN) <- Radar OUT (digital presence)

#include <Arduino.h>
#include <string.h>
#define LED_ENABLE 1
#if LED_ENABLE
#include <Adafruit_NeoPixel.h>
#endif

// Pin mapping based on user's wiring
static const int RADAR_TX_PIN = 14;   // ESP32 TX to radar RX
static const int RADAR_RX_PIN = 13;   // ESP32 RX from radar TX
static const int RADAR_OUT_PIN = 12;  // Radar OUT (digital)
// If your module exposes an ambient light output (often labeled "LIGHT"),
// wire it to an ADC-capable pin and set the pin number here. Set to -1 to disable.
static const int LIGHT_ADC_PIN = -1;   // e.g. 1, 2, 4, etc. for ESP32-S3 ADC-capable pins
// Onboard RGB LED (WS2812-style) data pin
static const int RGB_LED_PIN = 38;
static const int RGB_LED_COUNT = 1;

// Speeds
static const uint32_t USB_BAUD = 115200;     // Serial monitor
static const uint32_t RADAR_BAUD = 256000;   // LD2410 default UART

// Telemetry
static bool lastOutState = false;
static uint32_t lastBeatMs = 0;
static size_t uartBytesSinceBeat = 0;
static uint32_t lastSampleMs = 0;

// Optional: set to 1 to also dump raw hex frames
#define DUMP_HEX 0
// Output modes
#define ENABLE_PLOTTER 1    // Print plot-friendly numeric lines for Arduino Serial Plotter
#define ENABLE_HUMAN_LOGS 0 // Set to 1 for descriptive logs in Serial Monitor

// Parser buffer for radar UART
static uint8_t rxBuf[128];
static size_t rxLen = 0;

struct RadarSample {
  bool valid = false;
  uint8_t targetType = 0;   // bit0=moving, bit1=stationary
  uint16_t movingDist = 0;  // cm
  uint8_t movingEnergy = 0; // 0-100 typically
  uint16_t staticDist = 0;  // cm
  uint8_t staticEnergy = 0; // 0-100 typically
};

static RadarSample lastSample;

#if LED_ENABLE
static Adafruit_NeoPixel rgb(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

static inline uint8_t clamp8(int v) { return (v < 0) ? 0 : (v > 255 ? 255 : v); }

static void ledBegin()
{
  rgb.begin();
  rgb.setBrightness(40); // overall brightness (0-255)
  rgb.clear();
  rgb.show();
}

static void ledSetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}

static void updateLedFromStatus(const RadarSample &s, bool outState)
{
  // Determine presence/motion
  bool moving = s.valid && (s.targetType & 0x01);
  bool stationary = s.valid && (s.targetType & 0x02);
  bool presence = outState || moving || stationary;

  if (!presence) {
    ledSetRGB(0, 0, 0); // off
    return;
  }

  // Base colors
  uint8_t r = 0, g = 0, b = 0;
  uint8_t energy = 0;
  if (moving && !stationary) {
    // Moving only -> red
    r = 255; g = 16; b = 0; // a bit of orange tint
    energy = s.movingEnergy;
  } else if (!moving && stationary) {
    // Stationary only -> blue
    r = 0; g = 32; b = 255;
    energy = s.staticEnergy;
  } else if (moving && stationary) {
    // Both -> magenta
    r = 255; g = 0; b = 180;
    energy = max(s.movingEnergy, s.staticEnergy);
  } else {
    // OUT high but no parsed sample -> white dim
    r = g = b = 64;
    energy = 50;
  }

  // Scale intensity by energy (0..100)
  uint16_t scale = 32 + (uint16_t)energy * 223 / 100; // 32..255
  r = clamp8((int)r * scale / 255);
  g = clamp8((int)g * scale / 255);
  b = clamp8((int)b * scale / 255);

  // Optional ambient light dimming
  if (LIGHT_ADC_PIN >= 0) {
    int raw = analogRead(LIGHT_ADC_PIN); // 0..4095
    // In darkness, reduce brightness further to be gentle on eyes
    // Map 0..4095 -> 40%..100% scaling
    uint16_t ambScale = 102 + (uint32_t)raw * (255 - 102) / 4095; // 102..255
    r = clamp8((int)r * ambScale / 255);
    g = clamp8((int)g * ambScale / 255);
    b = clamp8((int)b * ambScale / 255);
  }

  ledSetRGB(r, g, b);
}
#endif

static void printSample(const RadarSample &s)
{
  if (!s.valid) return;
  const bool moving = (s.targetType & 0x01);
  const bool stationary = (s.targetType & 0x02);
  const bool presence = moving || stationary;

  #if ENABLE_HUMAN_LOGS
    Serial.printf("Radar: presence=%c moving=%c stationary=%c | ",
                  presence ? 'Y' : 'N', moving ? 'Y' : 'N', stationary ? 'Y' : 'N');

    if (moving) {
      Serial.printf("mov=%u.%02um E=%u ", s.movingDist / 100, s.movingDist % 100, s.movingEnergy);
    } else {
      Serial.print("mov=-- E=0 ");
    }
    if (stationary) {
      Serial.printf("stat=%u.%02um E=%u ", s.staticDist / 100, s.staticDist % 100, s.staticEnergy);
    } else {
      Serial.print("stat=-- E=0 ");
    }
    Serial.printf("| OUT=%d", (int)lastOutState);
    if (LIGHT_ADC_PIN >= 0) {
      int raw = analogRead(LIGHT_ADC_PIN);
      int pct = (raw * 100 + 2047) / 4095; // 0..100
      if (pct < 0) pct = 0; if (pct > 100) pct = 100;
      Serial.printf(" | Light=%d%%", pct);
    }
    Serial.println();
  #endif

  #if ENABLE_PLOTTER
    // Arduino Serial Plotter format: label:value pairs separated by tabs
    int mov_cm = moving ? (int)s.movingDist : -1;      // -1 means no reading
    int stat_cm = stationary ? (int)s.staticDist : -1; // -1 means no reading
    Serial.print("mov_cm:"); Serial.print(mov_cm);
    Serial.print("\tstat_cm:"); Serial.print(stat_cm);
    Serial.print("\tmovE:"); Serial.print((int)s.movingEnergy);
    Serial.print("\tstatE:"); Serial.print((int)s.staticEnergy);
    Serial.print("\tOUT:"); Serial.print((int)lastOutState);
    if (LIGHT_ADC_PIN >= 0) {
      int raw = analogRead(LIGHT_ADC_PIN);
      int pct = (raw * 100 + 2047) / 4095; // 0..100
      if (pct < 0) pct = 0; if (pct > 100) pct = 100;
      Serial.print("\tLight:"); Serial.print(pct);
    }
    Serial.println();
  #endif
}

// Try to extract and parse frames from rxBuf; returns true if any frame parsed
static bool parseFrames()
{
  bool any = false;
  size_t i = 0;
  while (i + 10 <= rxLen) {
    // find header F4 F3 F2 F1
    if (!(rxBuf[i] == 0xF4 && rxBuf[i+1] == 0xF3 && rxBuf[i+2] == 0xF2 && rxBuf[i+3] == 0xF1)) {
      ++i;
      continue;
    }
    if (i + 8 > rxLen) break; // need at least header+len
    uint16_t L = (uint16_t)rxBuf[i+4] | ((uint16_t)rxBuf[i+5] << 8);
    size_t frameLen = (size_t)L + 10; // 4 header + 2 len + L payload + 4 tail
    if (frameLen > sizeof(rxBuf)) {
      // drop this byte and resync
      ++i;
      continue;
    }
    if (i + frameLen > rxLen) break; // wait for more
    // verify tail F8 F7 F6 F5
    if (!(rxBuf[i + frameLen - 4] == 0xF8 && rxBuf[i + frameLen - 3] == 0xF7 &&
          rxBuf[i + frameLen - 2] == 0xF6 && rxBuf[i + frameLen - 1] == 0xF5)) {
      ++i; // bad alignment; resync
      continue;
    }

    // Got a full data frame
    const uint8_t *f = &rxBuf[i];
    // Optional hex dump of the frame
    if (DUMP_HEX) {
      for (size_t k = 0; k < frameLen; ++k) {
        uint8_t b = f[k];
        if (b < 16) Serial.print('0');
        Serial.print(b, HEX);
        Serial.print(' ');
      }
      Serial.println();
    }

    // Parse the common LD2410 data frame layout
    RadarSample s;
    if (L >= 13 && f[6] == 0x02 && f[7] == 0xAA && f[17] == 0x55 && f[18] == 0x00) {
      s.valid = true;
      s.targetType = f[8];
      s.movingDist = (uint16_t)f[9] | ((uint16_t)f[10] << 8);
      s.movingEnergy = f[11];
      s.staticDist = (uint16_t)f[12] | ((uint16_t)f[13] << 8);
      s.staticEnergy = f[14];
      // Clamp energies to 0-100 for readability
      if (s.movingEnergy > 100) s.movingEnergy = 100;
      if (s.staticEnergy > 100) s.staticEnergy = 100;
    }

    if (s.valid) {
      printSample(s);
      any = true;
      lastSample = s;
      lastSampleMs = millis();
      #if LED_ENABLE
        updateLedFromStatus(s, lastOutState);
      #endif
    }

    // consume this frame from the buffer
    size_t remain = rxLen - (i + frameLen);
    if (remain) memmove(rxBuf, &rxBuf[i + frameLen], remain);
    rxLen = remain;
    i = 0; // restart from beginning after compaction
  }

  // If we skipped to i>0 without consuming, shift left to discard noise
  if (i > 0 && i < rxLen) {
    memmove(rxBuf, &rxBuf[i], rxLen - i);
    rxLen -= i;
  } else if (i >= rxLen) {
    rxLen = 0;
  }

  return any;
}

void setup() {
  // USB serial for logging
  Serial.begin(USB_BAUD);
  // Give USB CDC a moment to connect (non-blocking)
  delay(200);

  // Radar digital OUT
  pinMode(RADAR_OUT_PIN, INPUT_PULLDOWN);

  // Radar UART (Serial2 on ESP32), set to RX=13, TX=14
  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  #if LED_ENABLE
    ledBegin();
  #endif

  // Initial banner
  #if ENABLE_HUMAN_LOGS
    Serial.println();
    Serial.println("LD2410B test on ESP32-S3");
    Serial.printf("USB Serial: %lu baud\n", (unsigned long)USB_BAUD);
    Serial.printf("Radar UART: %lu baud (RX=%d, TX=%d)\n", (unsigned long)RADAR_BAUD, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial.printf("Radar OUT pin: %d\n", RADAR_OUT_PIN);
  #endif
  if (LIGHT_ADC_PIN >= 0) {
    #if ENABLE_HUMAN_LOGS
      Serial.printf("Light ADC pin: %d\n", LIGHT_ADC_PIN);
    #endif
    analogReadResolution(12); // normalize to 12-bit (0..4095)
  } else {
    #if ENABLE_HUMAN_LOGS
      Serial.println("Light ADC pin: disabled (-1)");
    #endif
  }

  lastOutState = digitalRead(RADAR_OUT_PIN);
  #if ENABLE_HUMAN_LOGS
    Serial.printf("Initial OUT: %s\n", lastOutState ? "HIGH (target)" : "LOW (clear)");
  #endif

  lastBeatMs = millis();
}

void loop() {
  // Read any incoming UART bytes and feed parser
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (rxLen < sizeof(rxBuf)) {
      rxBuf[rxLen++] = b;
    } else {
      // simple overflow handling: drop oldest half to make room
      memmove(rxBuf, rxBuf + rxLen / 2, rxLen - rxLen / 2);
      rxLen = rxLen - rxLen / 2;
      rxBuf[rxLen++] = b;
    }
    ++uartBytesSinceBeat;
  }

  // Attempt to parse complete frames and print human-readable output
  parseFrames();

  // Report OUT pin state changes
  bool now = digitalRead(RADAR_OUT_PIN);
  if (now != lastOutState) {
    lastOutState = now;
    #if ENABLE_HUMAN_LOGS
      Serial.printf("OUT changed: %s at %lu ms\n", now ? "HIGH (target detected)" : "LOW (no target)", (unsigned long)millis());
    #endif
    #if LED_ENABLE
      // Update LED on OUT change using last known sample
      updateLedFromStatus(lastSample, lastOutState);
    #endif
  }

  // Heartbeat every 2 seconds with byte count
  uint32_t nowMs = millis();
  if (nowMs - lastBeatMs >= 2000) {
    #if ENABLE_HUMAN_LOGS
      if (LIGHT_ADC_PIN >= 0) {
        int raw = analogRead(LIGHT_ADC_PIN);
        int pct = (raw * 100 + 2047) / 4095;
        if (pct < 0) pct = 0; if (pct > 100) pct = 100;
        Serial.printf("Heartbeat: OUT=%d, Light=%d%%, radar UART bytes in 2s=%u\n", (int)lastOutState, pct, (unsigned)uartBytesSinceBeat);
      } else {
        Serial.printf("Heartbeat: OUT=%d, radar UART bytes in 2s=%u\n", (int)lastOutState, (unsigned)uartBytesSinceBeat);
      }
    #endif
    uartBytesSinceBeat = 0;
    lastBeatMs = nowMs;
  }

  // Light loop delay to keep logs readable
  delay(5);

  #if LED_ENABLE
  // If we haven't seen a frame for a while, still reflect OUT state
  if (millis() - lastSampleMs > 1000) {
    updateLedFromStatus(lastSample, lastOutState);
    lastSampleMs = millis();
  }
  #endif
}
