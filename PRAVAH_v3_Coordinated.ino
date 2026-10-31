/*
  PRAVAH v3 - Integrated Acoustic-Accelerometric Wearable Patch
  ---------------------------------------------------------------
  Purpose:
    Prototype firmware implementing the operating cycle described in the
    current PRAVAH research paper:
      - automatic patient-specific baseline calibration
      - day/night measurement schedule (74 scheduled measurement slots/day at defaults)
      - ~4.1 s acoustic measurement cycle
      - MPU6050 stillness check + bounded motion/signal-quality retries
      - no motion-specific OLED prompt during the configured night phase
      - 100-250 Hz low band and 550-750 Hz high band analysis
      - 600-800 Hz energy fraction
      - baseline-based acoustic anomaly detection
      - 10-minute persistence confirmation after an anomaly
      - OLED warnings/status
      - ESP32 deep sleep between scheduled checks

  IMPORTANT ENGINEERING NOTES:
    1) The paper does not specify a numerical frequency threshold or a
       numerical persistence fraction. This firmware therefore exposes
       prototype-tuning constants below; they are NOT clinical cutoffs.
    2) The firmware does not diagnose a clot/thrombus. It reports an abnormal
       AVF acoustic pattern that warrants clinical verification.
    3) The MAX4466 is treated as the analog acoustic sensor on GPIO34.
    4) The MPU6050 is used as the motion/thrill reference. The present code
       uses it for stillness/motion rejection and a low-frequency vibration
       metric; it does not claim full-band 100-800 Hz thrill spectroscopy.
    5) The ESP32 has no dedicated RTC module in the stated BOM. Standalone
       operation requires T HHMMSS (or T HHMM) after cold power-on. The desktop
       app sets the host computer time over USB and periodically resynchronizes it.
    6) Preferences stores the baseline in ESP32 NVS (non-volatile flash).
*/

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/i2s.h"
#include "esp_sleep.h"
#include "driver/adc.h"

// ----------------------------- Hardware -----------------------------
static constexpr int MIC_ADC_GPIO = 34;       // ADC1_CH6, input-only
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;

static constexpr uint8_t OLED_ADDR = 0x3C;
static constexpr int OLED_WIDTH = 128;
static constexpr int OLED_HEIGHT = 64;
static constexpr int OLED_RESET = -1;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
bool oledReady = false;

// MPU6050 registers
static constexpr uint8_t MPU_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t MPU_SMPLRT_DIV   = 0x19;
static constexpr uint8_t MPU_CONFIG       = 0x1A;
static constexpr uint8_t MPU_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t MPU_WHO_AM_I     = 0x75;

// ----------------------------- DSP -----------------------------
static constexpr uint32_t TARGET_SAMPLE_RATE = 8000;
static constexpr uint16_t FFT_SIZE = 4096;
static constexpr uint16_t I2S_BLOCK = 256;
static constexpr uint8_t WINDOWS_PER_MEASUREMENT = 8;   // 8 * 4096 / 8000 ~= 4.1 s
static constexpr uint8_t MIN_CLEAN_WINDOWS = 4;
static constexpr uint8_t MAX_MEASUREMENT_RETRIES = 3;  // 3 retries after the first attempt
static constexpr uint32_t MEASUREMENT_RETRY_DELAY_MS = 700;
static constexpr bool REQUIRE_MPU_FOR_VALID_MEASUREMENT = true;

// Patient-specific calibration settings (prototype heuristics; not clinical cutoffs).
static constexpr uint8_t CALIBRATION_SAMPLES = 10;
static constexpr uint8_t CALIBRATION_MIN_INLIERS = 7;
static constexpr float CALIBRATION_MAD_MULTIPLIER = 3.5f;
static constexpr float CALIBRATION_MIN_TOLERANCE_HZ = 20.0f;
static constexpr uint8_t CALIBRATION_MAX_ACQUISITION_ATTEMPTS_PER_BATCH = 12;
static constexpr uint8_t CALIBRATION_MAX_RETRIES_PER_ACQUISITION = 1;

static constexpr float ANALYSIS_MIN_HZ = 100.0f;
static constexpr float ANALYSIS_MAX_HZ = 1000.0f;
static constexpr float LOW_MIN_HZ = 100.0f;
static constexpr float LOW_MAX_HZ = 250.0f;
static constexpr float HIGH_MIN_HZ = 550.0f;
static constexpr float HIGH_MAX_HZ = 750.0f;
static constexpr float HIGH_EXT_MIN_HZ = 600.0f;
static constexpr float HIGH_EXT_MAX_HZ = 800.0f;

// Prototype anomaly settings: tune using your own dataset.
static constexpr float ACOUSTIC_FREQ_REL_TOLERANCE = 0.20f;  // prototype +/-20% baseline
static constexpr float ACOUSTIC_FREQ_MIN_TOLERANCE_HZ = 20.0f;
static constexpr float BASELINE_Z_THRESHOLD = 2.0f;

// Motion heuristic.
static constexpr float MOTION_DELTA_G = 0.06f;
static constexpr uint32_t PRECHECK_MS = 1000;
static constexpr uint32_t PRECHECK_SAMPLE_MS = 20;

// Confirmation mode.
static constexpr uint32_t CONFIRM_DURATION_MS = 10UL * 60UL * 1000UL;
static constexpr float CONFIRM_PERSISTENCE_FRACTION = 0.80f;
static constexpr float CONFIRM_FINAL_FRACTION = 0.75f;
static constexpr uint16_t CONFIRM_MIN_VALID_BLOCKS = 50;
static constexpr uint8_t CONFIRM_FINAL_BLOCKS = 8;
// Only call a signal "normalized" if normal-direction readings dominate both
// the full confirmation period and the most recent eight valid blocks.
static constexpr float CONFIRM_NORMALIZED_FRACTION = 0.75f;

// Low-power UI timing.
static constexpr uint32_t PRE_MEASURE_WAKE_MS = 1000;
static constexpr uint32_t RESULT_HOLD_MS = 5000;
static constexpr uint32_t WEB_CONTROL_TIMEOUT_MS = 90000; // app sends status every 10 s while idle

// Default schedule from the paper.
static constexpr uint16_t DEFAULT_DAY_START_MIN = 7 * 60 + 30;   // 07:30
static constexpr uint16_t DEFAULT_DAY_END_MIN   = 22 * 60 + 30;  // 22:30
static constexpr uint16_t DEFAULT_DAY_INTERVAL_MIN = 45;
static constexpr uint16_t DEFAULT_NIGHT_INTERVAL_MIN = 10;
uint16_t dayIntervalMin = DEFAULT_DAY_INTERVAL_MIN;
uint16_t nightIntervalMin = DEFAULT_NIGHT_INTERVAL_MIN;
static constexpr uint32_t SECONDS_PER_DAY = 86400UL;

// ----------------------------- Globals -----------------------------
Preferences prefs;

float realBuf[FFT_SIZE];
float imagBuf[FFT_SIZE];
uint16_t adcBuf[I2S_BLOCK];

struct Measurement {
  bool valid = false;
  bool motion = false;
  bool sensorError = false;
  uint8_t cleanWindows = 0;

  float acousticPeakHz = 0.0f;
  float highPeakHz = 0.0f;
  float lowPeakHz = 0.0f;

  float lowPeak = 0.0f;
  float highPeak = 0.0f;
  float highEnergyPct = 0.0f;
  float hlpr = 0.0f;

  float motionDeltaG = 0.0f;
  float effectiveSampleRateHz = 0.0f;
};

struct Baseline {
  bool valid = false;
  float acousticMeanHz = 0.0f;
  float acousticStdHz = 0.0f;
  float hlprMean = 0.0f;
  float hlprStd = 0.0f;
  float highPctMean = 0.0f;
  float highPctStd = 0.0f;
  uint16_t n = 0;
};

// Keep these types above all functions so the Arduino .ino auto-prototypes
// generated by the IDE can see them.
enum Direction {
  DIRECTION_NORMAL = 0,
  DIRECTION_ABOVE = 1,
  DIRECTION_BELOW = -1
};

struct ConfirmationResult {
  bool enoughData = false;
  bool persistent = false;
  bool normalized = false;
  uint16_t validBlocks = 0;
  float sameDirectionFraction = 0.0f;
  float finalFraction = 0.0f;
  float normalFraction = 0.0f;
  float finalNormalFraction = 0.0f;
};

// Biquad state must also be declared before all functions because the Arduino
// .ino preprocessor may generate prototypes before later struct definitions.
struct BiquadState {
  float z1 = 0.0f;
  float z2 = 0.0f;
};

Baseline baseline;

uint16_t dayStartMin = DEFAULT_DAY_START_MIN;
uint16_t dayEndMin = DEFAULT_DAY_END_MIN;

// RTC memory survives ESP32 deep sleep, but NOT a cold power removal/reset.
RTC_DATA_ATTR bool rtcClockValid = false;
RTC_DATA_ATTR uint32_t rtcScheduledTimeSec = 0;

bool autoCalibrationPending = false;
bool cycleRunning = false;
bool audioReady = false;
bool mpuReady = false;
String cmdBuffer;
uint32_t awakeBaseMillis = 0;

// USB local-control mode: keep the ESP32 awake while the desktop app is connected.
bool webControlEnabled = false;
bool webReleasePending = false;
bool standaloneSchedulePending = false;
bool webScheduleReady = false;
uint32_t webNextScheduledTargetSec = 0;
uint32_t webLastCommandMillis = 0;

bool isDayPhaseAt(uint32_t timeSec) {
  if (dayStartMin >= dayEndMin) return false;
  const uint32_t startSec = (uint32_t)dayStartMin * 60UL;
  const uint32_t endSec = (uint32_t)dayEndMin * 60UL;
  return timeSec >= startSec && timeSec < endSec;
}

bool isNightPhaseNow() {
  if (!rtcClockValid) return false;  // Unknown time: do not assume night.
  const uint32_t elapsedSec = (millis() - awakeBaseMillis) / 1000UL;
  const uint32_t nowSec = (rtcScheduledTimeSec + elapsedSec) % SECONDS_PER_DAY;
  return !isDayPhaseAt(nowSec);
}

// ----------------------------- Filter state -----------------------------
// 4th-order high-pass, 50 Hz, designed for the nominal 8 kHz sample rate.
// Literature support for this preprocessing should be cited in the paper;
// the firmware itself does not treat the filter as a clinical cutoff.
BiquadState hp1, hp2;

static constexpr float HP1_B0 = 0.94998178f;
static constexpr float HP1_B1 = -1.89996357f;
static constexpr float HP1_B2 = 0.94998178f;
static constexpr float HP1_A1 = -1.92850849f;
static constexpr float HP1_A2 = 0.92999644f;

static constexpr float HP2_B0 = 1.00000000f;
static constexpr float HP2_B1 = -2.00000000f;
static constexpr float HP2_B2 = 1.00000000f;
static constexpr float HP2_A1 = -1.96887750f;
static constexpr float HP2_A2 = 0.97039660f;

void resetHighpass() {
  hp1 = BiquadState();
  hp2 = BiquadState();
}

float biquad(float x, BiquadState &s,
             float b0, float b1, float b2, float a1, float a2) {
  const float y = b0 * x + s.z1;
  s.z1 = b1 * x - a1 * y + s.z2;
  s.z2 = b2 * x - a2 * y;
  return y;
}

float highpassSample(float x) {
  x = biquad(x, hp1, HP1_B0, HP1_B1, HP1_B2, HP1_A1, HP1_A2);
  x = biquad(x, hp2, HP2_B0, HP2_B1, HP2_B2, HP2_A1, HP2_A2);
  return x;
}

// ----------------------------- UI -----------------------------
void oledOff() {
  if (!oledReady) return;
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void oledOn() {
  if (!oledReady) return;
  display.ssd1306_command(SSD1306_DISPLAYON);
}

// Simple idle/start page: never show command syntax on the OLED.
void oledBrand() {
  if (!oledReady) return;
  oledOn();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(27, 24);
  display.print("PRAVAH");
  display.display();
}

void oledMessage(const String &line1,
                const String &line2 = "",
                const String &line3 = "",
                const String &line4 = "") {
  if (!oledReady) return;

  oledOn();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  display.println();
  display.setTextSize(2);
  display.println(line2);
  display.setTextSize(1);
  display.println();
  display.println(line3);
  display.println(line4);
  display.display();
}

void oledMeasurement(const Measurement &m, bool anomaly) {
  if (!oledReady) return;

  oledOn();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(anomaly ? "PRAVAH - WARNING" : "PRAVAH - AVF CHECK");
  display.setCursor(0, 14);
  display.print("Acoustic: ");
  display.print(m.acousticPeakHz, 1);
  display.println(" Hz");
  display.print("HLPR: ");
  display.println(m.hlpr, 3);
  display.print("High: ");
  display.print(m.highPeakHz, 1);
  display.println(" Hz");
  display.print("Motion: ");
  display.print(m.motionDeltaG, 3);
  display.println(" g");
  display.setCursor(0, 53);
  display.setTextSize(1);
  display.println(anomaly ? "ABNORMAL PATTERN" : "WITHIN BASELINE");
  display.display();
}

// ----------------------------- Logging -----------------------------
void logLine(const String &msg) {
  Serial.println(msg);
}

// ----------------------------- MPU6050 -----------------------------
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool mpuReadBytes(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom((int)MPU_ADDR, (int)len, (int)true);
  if (received != len) return false;

  for (uint8_t i = 0; i < len; ++i) {
    data[i] = Wire.read();
  }
  return true;
}

bool mpuBegin() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(50);

  // DLPF = 0 => widest accelerometer bandwidth (about 260 Hz).
  // We use the sensor primarily as a motion reference here.
  mpuWrite(MPU_PWR_MGMT_1, 0x00);
  delay(10);
  mpuWrite(MPU_CONFIG, 0x00);
  mpuWrite(MPU_SMPLRT_DIV, 79);   // 8 kHz / (79+1) = 100 Hz register sample rate
  mpuWrite(MPU_ACCEL_CONFIG, 0x00); // +/-2 g
  delay(10);

  uint8_t who = 0;
  if (!mpuReadBytes(MPU_WHO_AM_I, &who, 1)) return false;
  return (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x73);
}

bool readAccelG(float &ax, float &ay, float &az) {
  uint8_t d[6];
  if (!mpuReadBytes(MPU_ACCEL_XOUT_H, d, 6)) return false;

  int16_t x = (int16_t)((d[0] << 8) | d[1]);
  int16_t y = (int16_t)((d[2] << 8) | d[3]);
  int16_t z = (int16_t)((d[4] << 8) | d[5]);

  ax = (float)x / 16384.0f;
  ay = (float)y / 16384.0f;
  az = (float)z / 16384.0f;
  return true;
}

void mpuSleep() {
  if (mpuReady) mpuWrite(MPU_PWR_MGMT_1, 0x40);
}

bool precheckStillness(float &metricOut, bool &sensorErrorOut) {
  metricOut = 0.0f;
  sensorErrorOut = false;
  if (!mpuReady) {
    sensorErrorOut = true;
    return false;
  }

  float ax, ay, az;
  if (!readAccelG(ax, ay, az)) {
    sensorErrorOut = true;
    return false;
  }

  float prevAx = ax, prevAy = ay, prevAz = az;
  float sumDelta2 = 0.0f;
  uint16_t samples = 0;
  uint16_t pollAttempts = 0;
  uint16_t pollFailures = 0;
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < PRECHECK_MS) {
    delay(PRECHECK_SAMPLE_MS);
    pollAttempts++;
    if (!readAccelG(ax, ay, az)) {
      pollFailures++;
      continue;
    }

    const float dx = ax - prevAx;
    const float dy = ay - prevAy;
    const float dz = az - prevAz;
    const float delta = sqrtf(dx * dx + dy * dy + dz * dz);
    sumDelta2 += delta * delta;
    samples++;

    prevAx = ax;
    prevAy = ay;
    prevAz = az;
  }

  // Too few successful I2C reads means the motion check itself is unreliable;
  // do not mislabel a sensor fault as patient movement.
  if (samples < 10 || (pollAttempts > 0 && pollFailures * 2U > pollAttempts)) {
    sensorErrorOut = true;
    return false;
  }

  metricOut = sqrtf(sumDelta2 / (float)samples);
  return metricOut <= MOTION_DELTA_G;
}

// ----------------------------- FFT -----------------------------
void fft(float *re, float *im, uint16_t n) {
  for (uint16_t i = 1, j = 0; i < n; ++i) {
    uint16_t bit = n >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;
    if (i < j) {
      float tr = re[i]; re[i] = re[j]; re[j] = tr;
      float ti = im[i]; im[i] = im[j]; im[j] = ti;
    }
  }

  for (uint16_t len = 2; len <= n; len <<= 1) {
    const float angle = -2.0f * PI / (float)len;
    const float wLenRe = cosf(angle);
    const float wLenIm = sinf(angle);

    for (uint16_t i = 0; i < n; i += len) {
      float wRe = 1.0f;
      float wIm = 0.0f;
      const uint16_t half = len >> 1;

      for (uint16_t j = 0; j < half; ++j) {
        const uint16_t even = i + j;
        const uint16_t odd = even + half;

        const float tRe = wRe * re[odd] - wIm * im[odd];
        const float tIm = wRe * im[odd] + wIm * re[odd];

        const float uRe = re[even];
        const float uIm = im[even];

        re[even] = uRe + tRe;
        im[even] = uIm + tIm;
        re[odd] = uRe - tRe;
        im[odd] = uIm - tIm;

        const float nextWRe = wRe * wLenRe - wIm * wLenIm;
        const float nextWIm = wRe * wLenIm + wIm * wLenRe;
        wRe = nextWRe;
        wIm = nextWIm;
      }
    }
  }
}

float magnitudeAt(uint16_t k) {
  return sqrtf(realBuf[k] * realBuf[k] + imagBuf[k] * imagBuf[k]);
}

float refinedPeakHz(uint16_t k, float sampleRateHz) {
  const float center = magnitudeAt(k);
  const float left = (k > 0) ? magnitudeAt(k - 1) : center;
  const float right = (k + 1 < FFT_SIZE / 2) ? magnitudeAt(k + 1) : center;

  const float denom = left - 2.0f * center + right;
  float delta = 0.0f;
  if (fabsf(denom) > 1e-9f) {
    delta = 0.5f * (left - right) / denom;
    delta = constrain(delta, -0.5f, 0.5f);
  }

  const float refinedBin = (float)k + delta;
  return refinedBin * sampleRateHz / (float)FFT_SIZE;
}

// ----------------------------- I2S audio -----------------------------
bool audioI2SBegin() {
  i2s_config_t config = {};
  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
  config.sample_rate = TARGET_SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  config.communication_format = I2S_COMM_FORMAT_I2S_MSB;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 8;
  config.dma_buf_len = I2S_BLOCK;
  config.use_apll = false;
  config.tx_desc_auto_clear = false;
  config.fixed_mclk = 0;

  esp_err_t err = i2s_driver_install(I2S_PORT, &config, 0, nullptr);
  if (err != ESP_OK) {
    logLine(String("I2S install failed: ") + (int)err);
    return false;
  }

  err = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);
  if (err != ESP_OK) {
    logLine(String("I2S ADC mode failed: ") + (int)err);
    return false;
  }

  // ADC1_CH6 is GPIO34. Use the widest practical input range.
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

  i2s_adc_enable(I2S_PORT);
  i2s_zero_dma_buffer(I2S_PORT);
  return true;
}

bool captureAudioWindow(bool &motionDetected,
                        float &motionMetricG,
                        float &effectiveSampleRateHz,
                        bool &sensorErrorDetected) {
  motionDetected = false;
  motionMetricG = 0.0f;
  effectiveSampleRateHz = TARGET_SAMPLE_RATE;
  sensorErrorDetected = false;

  size_t collected = 0;
  float prevAx = 0.0f, prevAy = 0.0f, prevAz = 1.0f;
  bool havePrevAccel = false;
  float sumDelta2 = 0.0f;
  uint16_t motionSamples = 0;
  uint16_t motionReadAttempts = 0;
  uint16_t motionReadFailures = 0;
  uint32_t nextMotionReadMs = millis();

  const uint64_t captureStartUs = micros();

  while (collected < FFT_SIZE) {
    const size_t want = min((size_t)I2S_BLOCK, (size_t)(FFT_SIZE - collected));
    size_t bytesRead = 0;

    esp_err_t err = i2s_read(
      I2S_PORT,
      adcBuf,
      want * sizeof(uint16_t),
      &bytesRead,
      pdMS_TO_TICKS(1000)
    );

    if (err != ESP_OK || bytesRead == 0) return false;

    const size_t got = bytesRead / sizeof(uint16_t);

    for (size_t i = 0; i < got; ++i) {
      const float raw = (float)(adcBuf[i] & 0x0FFF);
      realBuf[collected + i] = highpassSample(raw);
      imagBuf[collected + i] = 0.0f;
    }
    collected += got;

    // Motion reference; roughly 25-50 Hz depending on I2S block timing.
    const uint32_t now = millis();
    if ((int32_t)(now - nextMotionReadMs) >= 0) {
      float ax, ay, az;
      motionReadAttempts++;
      if (mpuReady && readAccelG(ax, ay, az)) {
        if (havePrevAccel) {
          const float dx = ax - prevAx;
          const float dy = ay - prevAy;
          const float dz = az - prevAz;
          const float deltaG = sqrtf(dx * dx + dy * dy + dz * dz);
          sumDelta2 += deltaG * deltaG;
          motionSamples++;
        }
        prevAx = ax;
        prevAy = ay;
        prevAz = az;
        havePrevAccel = true;
      } else {
        motionReadFailures++;
      }
      nextMotionReadMs = now + 20;
    }
  }

  const uint64_t elapsedUs = micros() - captureStartUs;
  if (elapsedUs > 1000) {
    effectiveSampleRateHz = 1000000.0f * (float)FFT_SIZE / (float)elapsedUs;
  }

  // Require enough successful accelerometer reads to trust the motion estimate.
  // A failed or unreliable MPU6050 trace is a sensor-quality error, not motion.
  if (motionReadAttempts < 5 || motionSamples < 3 ||
      (motionReadAttempts > 0 && motionReadFailures * 2U > motionReadAttempts)) {
    sensorErrorDetected = true;
    return false;
  }

  motionMetricG = sqrtf(sumDelta2 / (float)motionSamples);
  motionDetected = (motionMetricG > MOTION_DELTA_G);

  // Remove any remaining mean bias.
  float mean = 0.0f;
  for (uint16_t i = 0; i < FFT_SIZE; ++i) mean += realBuf[i];
  mean /= (float)FFT_SIZE;

  // Hann window.
  for (uint16_t i = 0; i < FFT_SIZE; ++i) {
    const float x = realBuf[i] - mean;
    const float w = 0.5f * (1.0f - cosf(2.0f * PI * (float)i / (float)(FFT_SIZE - 1)));
    realBuf[i] = x * w;
    imagBuf[i] = 0.0f;
  }

  return true;
}

// ----------------------------- Measurement -----------------------------
Measurement takeMeasurement(bool quiet = false) {
  Measurement m;
  resetHighpass();

  float acousticFreqSum = 0.0f;
  float highPeakFreqSum = 0.0f;
  float lowPeakFreqSum = 0.0f;
  float lowPeakSum = 0.0f;
  float highPeakSum = 0.0f;
  float highPctSum = 0.0f;
  float hlprSum = 0.0f;
  float motionSum = 0.0f;
  float sampleRateSum = 0.0f;
  uint8_t clean = 0;
  bool anyMotion = false;

  if (!quiet) {
    oledMessage("AVF CHECK", "LISTENING...", "Capturing acoustic signal");
    logLine("[PRAVAH] Starting ~4.1 s acoustic capture. Keep arm still.");
  }

  for (uint8_t w = 0; w < WINDOWS_PER_MEASUREMENT; ++w) {
    bool motion = false;
    float motionMetric = 0.0f;
    float fsHz = TARGET_SAMPLE_RATE;

    bool windowSensorError = false;
    if (!captureAudioWindow(motion, motionMetric, fsHz, windowSensorError)) {
      m.valid = false;
      m.motion = anyMotion;
      m.sensorError = windowSensorError;
      return m;
    }

    motionSum += motionMetric;
    sampleRateSum += fsHz;

    if (motion) {
      anyMotion = true;
      if (!quiet) {
        logLine("[PRAVAH] Motion detected during measurement -> entire reading marked for RETAKE.");
      }
      // Do not partially salvage this measurement. The scheduled measurement
      // will be retried so a motion-corrupted reading does not consume a slot.
      continue;
    }

    fft(realBuf, imagBuf, FFT_SIZE);

    const uint16_t maxBin = (uint16_t)floorf(ANALYSIS_MAX_HZ * FFT_SIZE / fsHz);

    float lowPeak = 0.0f;
    float highPeak = 0.0f;
    float lowPeakHz = 0.0f;
    float highPeakHz = 0.0f;
    float dominant = 0.0f;
    float dominantHz = 0.0f;
    float totalEnergy = 0.0f;
    float highEnergy = 0.0f;

    for (uint16_t k = 1; k <= maxBin && k < FFT_SIZE / 2; ++k) {
      const float hz = k * fsHz / (float)FFT_SIZE;
      const float mag = magnitudeAt(k);
      const float energy = mag * mag;

      if (hz >= ANALYSIS_MIN_HZ && hz <= ANALYSIS_MAX_HZ) {
        totalEnergy += energy;
      }

      if (hz >= ANALYSIS_MIN_HZ && hz <= 800.0f && mag > dominant) {
        dominant = mag;
        dominantHz = refinedPeakHz(k, fsHz);
      }

      if (hz >= LOW_MIN_HZ && hz <= LOW_MAX_HZ && mag > lowPeak) {
        lowPeak = mag;
        lowPeakHz = refinedPeakHz(k, fsHz);
      }

      if (hz >= HIGH_MIN_HZ && hz <= HIGH_MAX_HZ && mag > highPeak) {
        highPeak = mag;
        highPeakHz = refinedPeakHz(k, fsHz);
      }

      if (hz >= HIGH_EXT_MIN_HZ && hz <= HIGH_EXT_MAX_HZ) {
        highEnergy += energy;
      }
    }

    // A valid feature set requires a real low-band peak and a dominant peak.
    if (lowPeak <= 0.0001f || dominantHz <= 0.0f) {
      continue;
    }

    const float hlpr = highPeak / lowPeak;
    const float highPct = (totalEnergy > 0.0f)
                        ? (100.0f * highEnergy / totalEnergy)
                        : 0.0f;

    if (!quiet) {
      logLine(String("[PRAVAH LIVE: freq=") + String(dominantHz, 1) +
              " Hz, high=" + String(highPeakHz, 1) +
              " Hz, hlpr=" + String(hlpr, 3) +
              ", energy=" + String(highPct, 1) + "%]");
    }

    acousticFreqSum += dominantHz;
    highPeakFreqSum += highPeakHz;
    lowPeakFreqSum += lowPeakHz;
    lowPeakSum += lowPeak;
    highPeakSum += highPeak;
    highPctSum += highPct;
    hlprSum += hlpr;
    clean++;
  }

  m.cleanWindows = clean;
  m.motionDeltaG = motionSum / (float)WINDOWS_PER_MEASUREMENT;
  m.effectiveSampleRateHz = sampleRateSum / (float)WINDOWS_PER_MEASUREMENT;

  // Any motion during any FFT window invalidates the entire measurement.
  // acquireCleanMeasurement() will retake it rather than counting it as a
  // completed scheduled reading.
  if (anyMotion) {
    m.valid = false;
    m.motion = true;
    return m;
  }

  if (clean < MIN_CLEAN_WINDOWS) {
    m.valid = false;
    m.motion = false;
    return m;
  }

  m.valid = true;
  m.motion = false;
  m.acousticPeakHz = acousticFreqSum / (float)clean;
  m.highPeakHz = highPeakFreqSum / (float)clean;
  m.lowPeakHz = lowPeakFreqSum / (float)clean;
  m.lowPeak = lowPeakSum / (float)clean;
  m.highPeak = highPeakSum / (float)clean;
  m.highEnergyPct = highPctSum / (float)clean;
  m.hlpr = hlprSum / (float)clean;
  return m;
}

// Runs the pre-check and, when needed, retakes the entire measurement.
// Retries do not create additional scheduled cycles; they are attempts to
// successfully complete the same scheduled measurement slot.
bool acquireCleanMeasurement(Measurement &m,
                             const char *context,
                             bool quiet = false,
                             uint8_t maxRetries = MAX_MEASUREMENT_RETRIES) {
  m = Measurement();

  if (REQUIRE_MPU_FOR_VALID_MEASUREMENT && !mpuReady) {
    m.sensorError = true;
    if (!quiet) logLine(String("[PRAVAH] ") + context + ": MPU6050 unavailable; measurement blocked.");
    return false;
  }

  for (uint8_t attempt = 0; attempt <= maxRetries; ++attempt) {
    if (!quiet) oledBrand();  // The accelerometer's one-second pre-check runs while PRAVAH is displayed.
    float preMotionMetric = 0.0f;
    bool precheckSensorError = false;
    const bool stationary = precheckStillness(preMotionMetric, precheckSensorError);
    if (precheckSensorError) {
      m = Measurement();
      m.sensorError = true;
      if (!quiet) logLine(String("[PRAVAH] ") + context + ": MPU6050 pre-check failed; not classifying it as movement.");
      return false;
    }
    if (!stationary) {
      m = Measurement();
      m.motion = true;
      if (!quiet) {
        logLine(String("[PRAVAH] ") + context + ": motion before capture, attempt " +
                String(attempt + 1) + "/" + String(maxRetries + 1));
      }
      if (attempt < maxRetries) {
        delay(MEASUREMENT_RETRY_DELAY_MS);
        continue;
      }
      return false;
    }

    m = takeMeasurement(quiet);
    if (m.sensorError) {
      if (!quiet) logLine(String("[PRAVAH] ") + context + ": MPU6050 data unreliable during capture.");
      return false;
    }
    if (m.valid && !m.motion) {
      return true;
    }

    if (!m.motion) {
      if (!quiet) logLine(String("[PRAVAH] ") + context + ": insufficient clean acoustic data; retrying the same slot.");
      if (attempt < maxRetries) {
        delay(MEASUREMENT_RETRY_DELAY_MS);
        continue;
      }
      return false;
    }

    if (!quiet) {
      logLine(String("[PRAVAH] ") + context + ": motion-corrupted reading -> retaking same scheduled slot.");
    }
    if (attempt < maxRetries) {
      delay(MEASUREMENT_RETRY_DELAY_MS);
    }
  }

  return false;
}

// ----------------------------- Baseline -----------------------------
float meanOf(const float *x, uint8_t n) {
  if (n == 0) return 0.0f;
  float sum = 0.0f;
  for (uint8_t i = 0; i < n; ++i) sum += x[i];
  return sum / (float)n;
}

float stdOf(const float *x, uint8_t n, float mean) {
  if (n < 2) return 0.01f;
  float sum = 0.0f;
  for (uint8_t i = 0; i < n; ++i) {
    const float d = x[i] - mean;
    sum += d * d;
  }
  const float v = sum / (float)(n - 1);
  return max(sqrtf(v), 0.01f);
}

float medianOf(const float *x, uint8_t n) {
  if (n == 0) return 0.0f;
  float tmp[CALIBRATION_SAMPLES];
  for (uint8_t i = 0; i < n; ++i) tmp[i] = x[i];
  for (uint8_t i = 1; i < n; ++i) {
    const float key = tmp[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      --j;
    }
    tmp[j + 1] = key;
  }
  if (n & 1U) return tmp[n / 2U];
  return 0.5f * (tmp[n / 2U - 1U] + tmp[n / 2U]);
}

float medianAbsoluteDeviation(const float *x, uint8_t n, float median) {
  if (n == 0) return 0.0f;
  float dev[CALIBRATION_SAMPLES];
  for (uint8_t i = 0; i < n; ++i) dev[i] = fabsf(x[i] - median);
  return medianOf(dev, n);
}

uint8_t selectCalibrationInliers(const float *acousticHz,
                                 uint8_t n,
                                 bool *keep) {
  const float med = medianOf(acousticHz, n);
  const float mad = medianAbsoluteDeviation(acousticHz, n, med);
  // 1.4826 converts MAD to a Gaussian-equivalent standard deviation.
  const float robustSigma = 1.4826f * mad;
  const float tolerance = max(CALIBRATION_MIN_TOLERANCE_HZ,
                              CALIBRATION_MAD_MULTIPLIER * robustSigma);

  uint8_t inliers = 0;
  for (uint8_t i = 0; i < n; ++i) {
    keep[i] = fabsf(acousticHz[i] - med) <= tolerance;
    if (keep[i]) inliers++;
  }
  return inliers;
}

void loadBaseline() {
  prefs.begin("pravah", true);
  baseline.valid = prefs.getBool("valid", false);
  baseline.acousticMeanHz = prefs.getFloat("fMean", 0.0f);
  baseline.acousticStdHz = prefs.getFloat("fStd", 0.0f);
  baseline.hlprMean = prefs.getFloat("hMean", 0.0f);
  baseline.hlprStd = prefs.getFloat("hStd", 0.0f);
  baseline.highPctMean = prefs.getFloat("pMean", 0.0f);
  baseline.highPctStd = prefs.getFloat("pStd", 0.0f);
  baseline.n = prefs.getUShort("n", 0);
  prefs.end();

  const bool sane = baseline.valid &&
                    baseline.n >= CALIBRATION_MIN_INLIERS &&
                    isfinite(baseline.acousticMeanHz) && baseline.acousticMeanHz > 0.0f &&
                    isfinite(baseline.acousticStdHz) && baseline.acousticStdHz > 0.0f &&
                    isfinite(baseline.hlprMean) && baseline.hlprMean >= 0.0f &&
                    isfinite(baseline.hlprStd) && baseline.hlprStd > 0.0f &&
                    isfinite(baseline.highPctMean) && baseline.highPctMean >= 0.0f && baseline.highPctMean <= 100.0f &&
                    isfinite(baseline.highPctStd) && baseline.highPctStd > 0.0f;

  if (!sane) {
    baseline = Baseline();
    logLine("[PRAVAH] No valid patient baseline stored.");
    return;
  }

  logLine(String("[PRAVAH] Baseline loaded: acoustic=") + String(baseline.acousticMeanHz, 1) +
          " +/- " + String(baseline.acousticStdHz, 1) + " Hz, n=" + baseline.n);
}

void saveBaseline() {
  prefs.begin("pravah", false);
  prefs.putBool("valid", baseline.valid);
  prefs.putFloat("fMean", baseline.acousticMeanHz);
  prefs.putFloat("fStd", baseline.acousticStdHz);
  prefs.putFloat("hMean", baseline.hlprMean);
  prefs.putFloat("hStd", baseline.hlprStd);
  prefs.putFloat("pMean", baseline.highPctMean);
  prefs.putFloat("pStd", baseline.highPctStd);
  prefs.putUShort("n", baseline.n);
  prefs.end();
}

bool collectCalibrationBatch(float *f, float *h, float *p, uint8_t n) {
  uint8_t count = 0;
  uint8_t acquisitionAttempts = 0;

  // Bound this loop: bad coupling or a disconnected sensor must not leave the
  // device calibrating forever. Three batch attempts are handled by the caller.
  while (count < n && acquisitionAttempts < CALIBRATION_MAX_ACQUISITION_ATTEMPTS_PER_BATCH) {
    acquisitionAttempts++;
    Measurement m;
    if (!acquireCleanMeasurement(m, "Calibration", true,
                                 CALIBRATION_MAX_RETRIES_PER_ACQUISITION)) {
      if (m.sensorError) {
        oledMessage("CALIBRATION", "SENSOR ERROR", "Check MPU6050 / wiring");
      } else if (m.motion) {
        if (isNightPhaseNow()) oledOff();
        else oledMessage("CALIBRATION", "RETRY", "Motion detected", "Keep arm still");
      } else {
        oledMessage("CALIBRATION", "RETRY", "Weak / invalid signal");
      }
      delay(MEASUREMENT_RETRY_DELAY_MS);
      continue;
    }

    f[count] = m.acousticPeakHz;
    h[count] = m.hlpr;
    p[count] = m.highEnergyPct;
    count++;

    if (isNightPhaseNow()) {
      oledMessage("CALIBRATION", String(count) + "/" + String(n),
                  String(m.acousticPeakHz, 1) + " Hz", "Reading collected");
    } else {
      oledMessage("CALIBRATION", String(count) + "/" + String(n),
                  String(m.acousticPeakHz, 1) + " Hz", "Keep arm still");
    }
    logLine(String("[PRAVAH] Baseline sample ") + count +
            ": acoustic=" + String(m.acousticPeakHz, 1) +
            " Hz, HLPR=" + String(m.hlpr, 3) +
            ", high%=" + String(m.highEnergyPct, 1));
    delay(500);
  }

  if (count < n) {
    logLine(String("[PRAVAH] Calibration batch incomplete: collected ") + count + "/" + n +
            " valid measurements after " + acquisitionAttempts + " acquisition attempts.");
    return false;
  }
  return true;
}

void calibrateBaseline() {
  if (!audioReady || (REQUIRE_MPU_FOR_VALID_MEASUREMENT && !mpuReady)) {
    oledMessage("CALIBRATION", "SENSOR ERROR", "Need MAX4466 + MPU6050");
    logLine("[PRAVAH] Calibration aborted: required sensor not ready.");
    return;
  }

  const uint8_t N = CALIBRATION_SAMPLES;
  float f[N], h[N], p[N];
  bool keep[N];

  if (isNightPhaseNow()) {
    oledMessage("CALIBRATION", "RUNNING", "Collecting readings");
  } else {
    oledMessage("CALIBRATION", "KEEP STILL", "Collecting clean readings", "Known healthy/reference state only");
  }
  logLine("=======================================================");
  logLine("[PRAVAH] AUTOMATIC BASELINE CALIBRATION");
  logLine("Use only with a clinically known healthy/reference AVF state.");

  // Retry the whole batch if it is too internally inconsistent to establish
  // a stable patient-specific baseline.
  for (uint8_t batchAttempt = 0; batchAttempt < 3; ++batchAttempt) {
    if (!collectCalibrationBatch(f, h, p, N)) {
      oledMessage("CALIBRATION", "BATCH FAILED", "Signal/sensor unavailable");
      delay(1000);
      continue;
    }
    const uint8_t inliers = selectCalibrationInliers(f, N, keep);

    if (inliers < CALIBRATION_MIN_INLIERS) {
      oledMessage("CALIBRATION", "UNSTABLE", "Readings vary too much", "Collecting a fresh batch");
      logLine(String("[PRAVAH] Calibration batch rejected: only ") + inliers +
              " / " + N + " acoustic inliers.");
      delay(1200);
      continue;
    }

    float fIn[CALIBRATION_SAMPLES];
    float hIn[CALIBRATION_SAMPLES];
    float pIn[CALIBRATION_SAMPLES];
    uint8_t j = 0;
    for (uint8_t i = 0; i < N; ++i) {
      if (!keep[i]) continue;
      fIn[j] = f[i];
      hIn[j] = h[i];
      pIn[j] = p[i];
      j++;
    }

    baseline.acousticMeanHz = meanOf(fIn, j);
    baseline.acousticStdHz = stdOf(fIn, j, baseline.acousticMeanHz);
    baseline.hlprMean = meanOf(hIn, j);
    baseline.hlprStd = stdOf(hIn, j, baseline.hlprMean);
    baseline.highPctMean = meanOf(pIn, j);
    baseline.highPctStd = stdOf(pIn, j, baseline.highPctMean);
    baseline.n = j;
    baseline.valid = true;
    saveBaseline();

    logLine(String("[PRAVAH] BASELINE SAVED: acoustic=") + String(baseline.acousticMeanHz, 1) +
            " +/- " + String(baseline.acousticStdHz, 1) +
            " Hz, retained=" + baseline.n + "/" + N);

    oledMessage("BASELINE SAVED", String(baseline.acousticMeanHz, 1) + " Hz",
                "Patient-specific baseline ready", String("Retained ") + j + "/" + N + " readings");
    delay(RESULT_HOLD_MS);
    oledOff();
    return;
  }

  baseline = Baseline();
  oledMessage("CALIBRATION", "FAILED", "Baseline not saved", "Collect again in a stable state");
  logLine("[PRAVAH] Calibration failed after 3 batches; no baseline saved.");
  delay(RESULT_HOLD_MS);
  oledOff();
}

void eraseBaseline() {
  baseline = Baseline();
  prefs.begin("pravah", false);
  // Remove only baseline keys. Do NOT clear the namespace because the same
  // namespace also stores the user's day/night schedule.
  prefs.remove("valid");
  prefs.remove("fMean");
  prefs.remove("fStd");
  prefs.remove("hMean");
  prefs.remove("hStd");
  prefs.remove("pMean");
  prefs.remove("pStd");
  prefs.remove("n");
  prefs.end();
  logLine("[PRAVAH] Stored baseline erased; schedule retained.");
  oledMessage("BASELINE", "ERASED", "Automatic calibration will run next.");
  delay(RESULT_HOLD_MS);
  oledOff();
}

// ----------------------------- Anomaly logic -----------------------------
float baselineAcousticToleranceHz() {
  if (!baseline.valid) return 0.0f;
  const float relTol = fabsf(baseline.acousticMeanHz) * ACOUSTIC_FREQ_REL_TOLERANCE;
  const float zTol = BASELINE_Z_THRESHOLD * max(baseline.acousticStdHz, 0.01f);
  return max(ACOUSTIC_FREQ_MIN_TOLERANCE_HZ, max(relTol, zTol));
}

Direction frequencyDirection(const Measurement &m) {
  if (!baseline.valid || !m.valid) return DIRECTION_NORMAL;
  const float tol = baselineAcousticToleranceHz();
  if (m.acousticPeakHz > baseline.acousticMeanHz + tol) return DIRECTION_ABOVE;
  if (m.acousticPeakHz < baseline.acousticMeanHz - tol) return DIRECTION_BELOW;
  return DIRECTION_NORMAL;
}

Direction spectralDirection(const Measurement &m) {
  if (!baseline.valid || !m.valid) return DIRECTION_NORMAL;

  const float hTol = BASELINE_Z_THRESHOLD * max(baseline.hlprStd, 0.01f);
  const float pTol = BASELINE_Z_THRESHOLD * max(baseline.highPctStd, 0.01f);

  const float hlprDelta = m.hlpr - baseline.hlprMean;
  const float highPctDelta = m.highEnergyPct - baseline.highPctMean;

  const float hlprZ = hlprDelta / max(baseline.hlprStd, 0.01f);
  const float highPctZ = highPctDelta / max(baseline.highPctStd, 0.01f);

  const float positiveScore = max(hlprZ, highPctZ);
  const float negativeScore = min(hlprZ, highPctZ);

  if ((hlprDelta >= hTol || highPctDelta >= pTol) && positiveScore >= BASELINE_Z_THRESHOLD) {
    return DIRECTION_ABOVE;
  }

  if ((hlprDelta <= -hTol || highPctDelta <= -pTol) && negativeScore <= -BASELINE_Z_THRESHOLD) {
    return DIRECTION_BELOW;
  }

  return DIRECTION_NORMAL;
}

Direction anomalyDirection(const Measurement &m) {
  // The dominant acoustic-frequency deviation is the primary decision feature.
  const Direction frequencyDir = frequencyDirection(m);
  if (frequencyDir != DIRECTION_NORMAL) return frequencyDir;

  // Secondary spectral features are now checked in both directions rather than
  // treating only an increase as abnormal.
  return spectralDirection(m);
}

void logMeasurement(const Measurement &m, const char *measurementType) {
  logLine("---------------- PRAVAH READING ----------------");
  logLine(String("Measurement type: ") + measurementType);
  logLine(String("Dominant acoustic frequency: ") + String(m.acousticPeakHz, 1) + " Hz");
  logLine(String("Low-band peak:              ") + String(m.lowPeak, 2) +
          " @ " + String(m.lowPeakHz, 1) + " Hz");
  logLine(String("High-band peak:             ") + String(m.highPeak, 2) +
          " @ " + String(m.highPeakHz, 1) + " Hz");
  logLine(String("HLPR:                       ") + String(m.hlpr, 3));
  logLine(String("600-800 Hz energy:          ") + String(m.highEnergyPct, 1) + " %");
  logLine(String("Motion metric:              ") + String(m.motionDeltaG, 3) + " g delta");
  logLine(String("Clean FFT windows:          ") + m.cleanWindows);
  logLine(String("Measured sample rate:        ") + String(m.effectiveSampleRateHz, 1) + " Hz");

  if (baseline.valid) {
    const float z = (m.acousticPeakHz - baseline.acousticMeanHz) /
                    max(baseline.acousticStdHz, 0.01f);
    logLine(String("Acoustic baseline z-score:   ") + String(z, 2));
  }
}

// ----------------------------- Confirmation mode -----------------------------
ConfirmationResult confirmAnomaly(Direction initialDirection) {
  ConfirmationResult r;
  if (initialDirection == DIRECTION_NORMAL) return r;

  if (isNightPhaseNow()) {
    oledMessage("WARNING", "CONFIRMING", "Tracking for 10 minutes");
  } else {
    oledMessage("WARNING", "CONFIRMING", "Tracking for 10 minutes", "Keep arm still when possible");
  }
  logLine("[PRAVAH] ANOMALY CONFIRMATION: 10-minute repeated clean monitoring.");

  uint16_t sameDir = 0;
  uint16_t normalCount = 0;
  uint16_t valid = 0;
  Direction finalDirs[CONFIRM_FINAL_BLOCKS];
  for (uint8_t i = 0; i < CONFIRM_FINAL_BLOCKS; ++i) finalDirs[i] = DIRECTION_NORMAL;
  uint8_t finalIndex = 0;

  const uint32_t startMs = millis();
  uint32_t lastUiMs = 0;

  while ((uint32_t)(millis() - startMs) < CONFIRM_DURATION_MS) {
    Measurement m;
    const bool gotClean = acquireCleanMeasurement(m, "Confirmation", true);

    if (gotClean) {
      const Direction d = anomalyDirection(m);
      valid++;
      if (d == initialDirection) sameDir++;
      if (d == DIRECTION_NORMAL) normalCount++;

      finalDirs[finalIndex % CONFIRM_FINAL_BLOCKS] = d;
      finalIndex++;

      if (valid <= 4) {
        logLine(String("[PRAVAH] Confirm block: acoustic=") + String(m.acousticPeakHz, 1) +
                " Hz direction=" + (int)d);
      }
    } else if (m.motion) {
      logLine("[PRAVAH] Confirmation slot had motion; same block was retried and not counted.");
    }

    if ((uint32_t)(millis() - lastUiMs) >= 30000UL) {
      lastUiMs = millis();
      const uint32_t elapsed = millis() - startMs;
      const uint32_t remain = (elapsed < CONFIRM_DURATION_MS)
                            ? (CONFIRM_DURATION_MS - elapsed)
                            : 0;
      const uint16_t remainSec = (uint16_t)(remain / 1000UL);
      oledMessage("CONFIRMING", String(remainSec / 60) + ":" + String(remainSec % 60),
                  String("Valid blocks: ") + valid,
                  isNightPhaseNow() ? "Monitoring signal" : "Keep still if possible");
    }
  }

  r.validBlocks = valid;
  r.enoughData = (valid >= CONFIRM_MIN_VALID_BLOCKS);

  if (valid > 0) {
    r.sameDirectionFraction = (float)sameDir / (float)valid;
  }

  // finalDirs is a circular buffer. When more than 8 entries exist,
  // finalIndex % 8 is the oldest entry among the final 8 stored blocks.
  const uint8_t finalCount = min((uint8_t)CONFIRM_FINAL_BLOCKS, finalIndex);
  uint8_t finalSame = 0;
  uint8_t finalNormal = 0;
  if (finalCount > 0) {
    const uint8_t start = (finalIndex >= CONFIRM_FINAL_BLOCKS)
                        ? (finalIndex % CONFIRM_FINAL_BLOCKS)
                        : 0;
    for (uint8_t i = 0; i < finalCount; ++i) {
      const uint8_t idx = (start + i) % CONFIRM_FINAL_BLOCKS;
      if (finalDirs[idx] == initialDirection) finalSame++;
      if (finalDirs[idx] == DIRECTION_NORMAL) finalNormal++;
    }
    r.finalFraction = (float)finalSame / (float)finalCount;
    r.finalNormalFraction = (float)finalNormal / (float)finalCount;
  }
  if (valid > 0) r.normalFraction = (float)normalCount / (float)valid;

  r.persistent = r.enoughData &&
                 (r.sameDirectionFraction >= CONFIRM_PERSISTENCE_FRACTION) &&
                 (r.finalFraction >= CONFIRM_FINAL_FRACTION);

  // Normalized now means the measured features actually returned within the
  // configured baseline limits for most blocks, including the latest blocks.
  // A mixed/non-persistent pattern is NOT automatically called normalized.
  r.normalized = r.enoughData && !r.persistent &&
                 (r.normalFraction >= CONFIRM_NORMALIZED_FRACTION) &&
                 (r.finalNormalFraction >= CONFIRM_NORMALIZED_FRACTION);

  logLine(String("[PRAVAH] Confirmation fractions: same-direction=") +
          String(r.sameDirectionFraction * 100.0f, 1) + "%, normal=" +
          String(r.normalFraction * 100.0f, 1) + "%, final-normal=" +
          String(r.finalNormalFraction * 100.0f, 1) + "%");

  if (r.persistent) {
    logLine("[PRAVAH] Confirmation result: persistent abnormal pattern.");
    oledMessage("PERSISTENT", "ANOMALY", "Abnormal access signal persisted", "Clinical verification required");
  } else if (r.normalized) {
    logLine("[PRAVAH] Confirmation result: readings returned within baseline limits.");
    oledMessage("SIGNAL", "NORMAL", "Within baseline again", "Initial change not sustained");
  } else if (r.enoughData) {
    logLine("[PRAVAH] Confirmation result: inconclusive pattern; neither persistent nor clearly normal.");
    oledMessage("INCONCLUSIVE", "RECHECK", "No clear persistence or return", "Retest next cycle");
  } else {
    logLine("[PRAVAH] Confirmation result: insufficient valid data.");
    oledMessage("LOW", "CONFIDENCE", "Confirmation data insufficient", "Retest cleanly");
  }

  return r;
}

// ----------------------------- Virtual clock / schedule -----------------------------
uint32_t getLogicalNowSec() {
  if (!rtcClockValid) return 0;
  const uint32_t awakeElapsedSec = (millis() - awakeBaseMillis) / 1000UL;
  return (rtcScheduledTimeSec + awakeElapsedSec) % SECONDS_PER_DAY;
}

uint32_t forwardSeconds(uint32_t fromSec, uint32_t toSec) {
  if (toSec >= fromSec) return toSec - fromSec;
  return SECONDS_PER_DAY - fromSec + toSec;
}

bool validSchedule() {
  return dayStartMin < dayEndMin;
}

uint32_t nextScheduledTargetSec(uint32_t nowSec) {
  if (!validSchedule()) return (nowSec + nightIntervalMin * 60UL) % SECONDS_PER_DAY;

  const uint32_t dayStartSec = (uint32_t)dayStartMin * 60UL;
  const uint32_t dayEndSec = (uint32_t)dayEndMin * 60UL;

  if (nowSec >= dayStartSec && nowSec < dayEndSec) {
    const uint32_t elapsed = nowSec - dayStartSec;
    const uint32_t interval = dayIntervalMin * 60UL;
    const uint32_t slotsAhead = elapsed / interval + 1UL;
    uint32_t target = dayStartSec + slotsAhead * interval;
    if (target > dayEndSec) target = dayEndSec;
    return target % SECONDS_PER_DAY;
  }

  const uint32_t interval = nightIntervalMin * 60UL;
  const uint32_t elapsed = forwardSeconds(dayEndSec, nowSec);
  const uint32_t slotsAhead = elapsed / interval + 1UL;
  const uint32_t target = (dayEndSec + slotsAhead * interval) % SECONDS_PER_DAY;
  return target;
}

String formatTimeSec(uint32_t sec) {
  sec %= SECONDS_PER_DAY;
  const uint16_t h = sec / 3600U;
  const uint16_t m = (sec % 3600U) / 60U;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02u:%02u", h, m);
  return String(buf);
}

uint16_t parseHHMM(const String &s) {
  if (s.length() != 4) return 65535;
  for (uint8_t i = 0; i < 4; ++i) {
    if (!isDigit(s[i])) return 65535;
  }
  const uint16_t hh = (uint16_t)(s[0] - '0') * 10 + (uint16_t)(s[1] - '0');
  const uint16_t mm = (uint16_t)(s[2] - '0') * 10 + (uint16_t)(s[3] - '0');
  if (hh > 23 || mm > 59) return 65535;
  return hh * 60U + mm;
}

void saveSchedule() {
  prefs.begin("pravah", false);
  prefs.putUShort("dayStart", dayStartMin);
  prefs.putUShort("dayEnd", dayEndMin);
  prefs.putUShort("dayInt", dayIntervalMin);
  prefs.putUShort("nightInt", nightIntervalMin);
  prefs.end();
}

void loadSchedule() {
  prefs.begin("pravah", true);
  dayStartMin = prefs.getUShort("dayStart", DEFAULT_DAY_START_MIN);
  dayEndMin = prefs.getUShort("dayEnd", DEFAULT_DAY_END_MIN);
  dayIntervalMin = prefs.getUShort("dayInt", DEFAULT_DAY_INTERVAL_MIN);
  nightIntervalMin = prefs.getUShort("nightInt", DEFAULT_NIGHT_INTERVAL_MIN);
  prefs.end();
  if (!validSchedule()) {
    dayStartMin = DEFAULT_DAY_START_MIN;
    dayEndMin = DEFAULT_DAY_END_MIN;
  }
  if (dayIntervalMin < 1 || dayIntervalMin > 720) dayIntervalMin = DEFAULT_DAY_INTERVAL_MIN;
  if (nightIntervalMin < 1 || nightIntervalMin > 720) nightIntervalMin = DEFAULT_NIGHT_INTERVAL_MIN;
}

void sleepUntilNextScheduledSlot() {
  if (!rtcClockValid) return;
  if (webControlEnabled) {
    oledOff();
    logLine("[PRAVAH] Web control active: keeping ESP32 awake.");
    return;
  }

  const uint32_t nowSec = getLogicalNowSec();
  const bool quietNight = !isDayPhaseAt(nowSec);
  const uint32_t targetSec = nextScheduledTargetSec(nowSec);
  const uint32_t deltaSec = forwardSeconds(nowSec, targetSec);

  // Store the exact scheduled target as the next logical wake time.
  rtcScheduledTimeSec = targetSec;

  // deltaSec is already measured from the current logical time; do not subtract
  // the entire awake duration again, or the device would wake too early.
  const int64_t desiredWakeMs = (int64_t)deltaSec * 1000LL;
  int64_t sleepMs = desiredWakeMs - (int64_t)PRE_MEASURE_WAKE_MS;

  if (sleepMs < 1000LL) sleepMs = 1000LL;

  logLine(String("[PRAVAH] Current time: ") + formatTimeSec(nowSec));
  logLine(String("[PRAVAH] Next scheduled measurement: ") + formatTimeSec(targetSec));
  Serial.printf("[PRAVAH] Sleep: %llu s\n", (unsigned long long)(sleepMs / 1000LL));

  // Result screen has already been shown for RESULT_HOLD_MS. Keep the OLED dark while sleeping.
  (void)quietNight;
  oledOff();
  mpuSleep();
  if (audioReady) i2s_adc_disable(I2S_PORT);

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
  Serial.flush();
  esp_deep_sleep_start();
}

// ----------------------------- Serial commands -----------------------------
void printHelp() {
  Serial.println();
  Serial.println("PRAVAH COMMANDS");
  Serial.println("  T HHMMSS = set current time with seconds, e.g. T 183045; T HHMM also accepted");
  Serial.println("  S HHMM   = set day-phase start, e.g. S 0730");
  Serial.println("  E HHMM   = set day-phase end, e.g. E 2230");
  Serial.println("  D minutes = set daytime interval, 1..720");
  Serial.println("  N minutes = set nighttime interval, 1..720");
  Serial.println("  W ON/OFF  = enable/disable connected local-app control");
  Serial.println("  R         = restore default schedule (baseline retained)");
  Serial.println("  M        = manual measurement now");
  Serial.println("  B        = re-calibrate healthy baseline");
  Serial.println("  X        = erase stored baseline");
  Serial.println("  P        = print status");
  Serial.println("  H        = help");
  Serial.println();
}

void printStatus() {
  Serial.println("================ PRAVAH STATUS ================");
  Serial.println(String("Clock valid: ") + (rtcClockValid ? "YES" : "NO"));
  if (rtcClockValid) Serial.println(String("Current logical time: ") + formatTimeSec(getLogicalNowSec()));
  Serial.println(String("Day start: ") + formatTimeSec((uint32_t)dayStartMin * 60UL));
  Serial.println(String("Day end: ") + formatTimeSec((uint32_t)dayEndMin * 60UL));
  Serial.println(String("Day interval: ") + dayIntervalMin + " min");
  Serial.println(String("Night interval: ") + nightIntervalMin + " min");
  Serial.println(String("Web control: ") + (webControlEnabled ? "ON" : "OFF"));
  Serial.println(String("Baseline: ") + (baseline.valid ? "VALID" : "NOT SET"));
  if (baseline.valid) {
    Serial.println(String("Acoustic baseline: ") + String(baseline.acousticMeanHz, 1) +
                   " +/- " + String(baseline.acousticStdHz, 1) + " Hz");
  }
  Serial.println(String("Audio: ") + (audioReady ? "OK" : "ERROR"));
  Serial.println(String("MPU6050: ") + (mpuReady ? "OK" : "ERROR"));
  Serial.println(String("OLED: ") + (oledReady ? "OK" : "ERROR"));
  Serial.println("================================================");
}

void maybeRunAutomaticCalibration() {
  if (cycleRunning || !rtcClockValid || baseline.valid || !audioReady) return;
  cycleRunning = true;
  calibrateBaseline();
  cycleRunning = false;

  if (baseline.valid) {
    sleepUntilNextScheduledSlot();
  }
}

void manualMeasurement() {
  if (!audioReady) {
    oledMessage("AUDIO ERROR", "CHECK MAX4466", "GPIO34 / power / wiring");
    delay(RESULT_HOLD_MS);
    oledOff();
    return;
  }

  cycleRunning = true;
  logLine("[PRAVAH] Manual measurement starting.");

  Measurement m;
  if (!acquireCleanMeasurement(m, "Manual", false)) {
    if (m.sensorError) {
      oledMessage("SENSOR ERROR", "CHECK MPU6050", "Measurement not completed");
    } else if (m.motion) {
      if (isNightPhaseNow()) oledOff();
      else oledMessage("MEASUREMENT", "RETRY LIMIT", "Motion persisted", "Manual check not completed");
    } else {
      oledMessage("LOW", "CONFIDENCE", "Weak / invalid acoustic signal");
    }
    delay(RESULT_HOLD_MS);
    oledOff();
    cycleRunning = false;
    return;
  }

  logMeasurement(m, "Manual");
  if (!baseline.valid) {
    logLine("[PRAVAH] No baseline - measurement cannot be compared.");
    oledMessage("NO BASELINE", "NOT COMPARED", "Calibrate first", "No clinical conclusion");
  } else {
    const Direction d = anomalyDirection(m);
    const bool anomaly = (d != DIRECTION_NORMAL);
    oledMeasurement(m, anomaly);
    if (anomaly) {
      logLine("STATUS: ABNORMAL ACCESS SIGNAL - ENTERING 10 MIN CONFIRMATION");
      delay(RESULT_HOLD_MS);
      ConfirmationResult c = confirmAnomaly(d);
      (void)c;
    } else {
      logLine("STATUS: WITHIN PATIENT BASELINE");
    }
  }

  delay(RESULT_HOLD_MS);
  oledOff();
  cycleRunning = false;
}

void automaticMeasurementCycle() {
  if (cycleRunning || !rtcClockValid || !baseline.valid || !audioReady) return;

  cycleRunning = true;
  logLine("[PRAVAH] Scheduled measurement starting.");

  Measurement m;
  if (!acquireCleanMeasurement(m, "Scheduled", false)) {
    if (m.sensorError) {
      logLine("[PRAVAH] Scheduled measurement failed: MPU6050/sensor data unreliable.");
      oledMessage("SENSOR ERROR", "CHECK MPU6050", "Measurement not completed");
    } else if (m.motion) {
      logLine("[PRAVAH] Motion persisted after retry limit; scheduled slot not completed.");
      if (isNightPhaseNow()) oledOff();
      else oledMessage("KEEP ARM", "STILL", "Retry limit reached", "Next scheduled slot will retry");
    } else {
      logLine("[PRAVAH] Scheduled measurement invalid: weak / invalid acoustic data.");
      oledMessage("LOW", "CONFIDENCE", "Weak acoustic signal", "Next scheduled slot will retry");
    }
    delay(RESULT_HOLD_MS);
    oledOff();
    cycleRunning = false;
    sleepUntilNextScheduledSlot();
    return;
  }

  logMeasurement(m, "Scheduled");
  const Direction d = anomalyDirection(m);
  const bool anomaly = (d != DIRECTION_NORMAL);

  if (!anomaly) {
    logLine("STATUS: WITHIN PATIENT BASELINE");
    oledMeasurement(m, false);
    delay(RESULT_HOLD_MS);
    oledOff();
  } else {
    logLine("STATUS: ABNORMAL ACCESS SIGNAL - ENTERING 10 MIN CONFIRMATION");
    oledMeasurement(m, true);
    delay(RESULT_HOLD_MS);
    ConfirmationResult c = confirmAnomaly(d);
    (void)c;
    delay(RESULT_HOLD_MS);
    oledOff();
  }

  cycleRunning = false;
  sleepUntilNextScheduledSlot();
}

void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (webControlEnabled) webLastCommandMillis = millis();

  String u = line;
  u.toUpperCase();

  if (u == "H") {
    printHelp();
    return;
  }

  if (u == "P") {
    printStatus();
    return;
  }

  if (u == "M") {
    manualMeasurement();
    return;
  }

  if (u == "B") {
    if (!rtcClockValid) {
      Serial.println("Set the time first with T HHMMSS (or T HHMM).");
      oledBrand();
      return;
    }
    cycleRunning = true;
    calibrateBaseline();
    cycleRunning = false;
    return;
  }

  if (u == "X") {
    eraseBaseline();
    autoCalibrationPending = rtcClockValid;
    return;
  }

  if (u == "W ON" || u == "W OFF") {
    webControlEnabled = (u == "W ON");
    webLastCommandMillis = millis();
    webReleasePending = !webControlEnabled && rtcClockValid;
    if (!webControlEnabled && rtcClockValid && !baseline.valid) autoCalibrationPending = true;
    webScheduleReady = false;
    Serial.println(String("Web control: ") + (webControlEnabled ? "ON" : "OFF"));
    logLine(String("[PRAVAH] Web control ") + (webControlEnabled ? "enabled; keeping awake." : "disabled; standalone scheduling requested."));
    return;
  }

  if (u == "R") {
    dayStartMin = DEFAULT_DAY_START_MIN;
    dayEndMin = DEFAULT_DAY_END_MIN;
    dayIntervalMin = DEFAULT_DAY_INTERVAL_MIN;
    nightIntervalMin = DEFAULT_NIGHT_INTERVAL_MIN;
    saveSchedule();
    webScheduleReady = false;
    logLine("[PRAVAH] Schedule reset to defaults; stored baseline retained.");
    printStatus();
    return;
  }

  if (u.startsWith("D") || u.startsWith("N")) {
    const bool isDayInterval = u.startsWith("D");
    String value = u.substring(1);
    value.trim();
    if (value.length() == 0) {
      Serial.println("Interval command requires whole minutes, e.g. D 45 or N 10.");
      return;
    }
    for (uint16_t i = 0; i < value.length(); ++i) {
      if (!isDigit(value[i])) {
        Serial.println("Invalid interval. Use whole minutes from 1 to 720.");
        return;
      }
    }
    const long val = value.toInt();
    if (val < 1 || val > 720) {
      Serial.println("Invalid interval. Use whole minutes from 1 to 720.");
      return;
    }
    if (isDayInterval) dayIntervalMin = (uint16_t)val;
    else nightIntervalMin = (uint16_t)val;
    saveSchedule();
    webScheduleReady = false;
    Serial.println(String(isDayInterval ? "Day interval: " : "Night interval: ") + val + " min");
    return;
  }

  if (u.startsWith("T")) {
    String value = u.substring(1);
    value.trim();
    uint16_t mins = 65535;
    uint16_t seconds = 0;
    if (value.length() == 4) {
      mins = parseHHMM(value);
    } else if (value.length() == 6) {
      String hhmm = value.substring(0, 4);
      String ss = value.substring(4, 6);
      mins = parseHHMM(hhmm);
      if (ss.length() != 2 || !isDigit(ss[0]) || !isDigit(ss[1])) mins = 65535;
      else seconds = (uint16_t)(ss[0] - '0') * 10U + (uint16_t)(ss[1] - '0');
      if (seconds > 59) mins = 65535;
    }
    if (mins == 65535) {
      Serial.println("Use T HHMMSS (e.g. T 183045) or T HHMM (e.g. T 1830).");
      return;
    }

    rtcScheduledTimeSec = (uint32_t)mins * 60UL + seconds;
    rtcClockValid = true;
    awakeBaseMillis = millis();
    autoCalibrationPending = !baseline.valid;
    if (webControlEnabled && !webScheduleReady) webScheduleReady = false;
    if (!webControlEnabled) standaloneSchedulePending = baseline.valid;

    char clockBuf[9];
    snprintf(clockBuf, sizeof(clockBuf), "%02u:%02u:%02u",
             (unsigned)(rtcScheduledTimeSec / 3600UL),
             (unsigned)((rtcScheduledTimeSec % 3600UL) / 60UL),
             (unsigned)(rtcScheduledTimeSec % 60UL));
    Serial.println(String("[PRAVAH] Time set to ") + String(clockBuf));
    return;
  }

  if (u.startsWith("S")) {
    String value = u.substring(1);
    value.trim();
    uint16_t mins = parseHHMM(value);
    if (mins == 65535 || mins >= dayEndMin) {
      Serial.println("Invalid day start. It must be HHMM and before day end.");
      return;
    }
    dayStartMin = mins;
    saveSchedule();
    webScheduleReady = false;
    Serial.println(String("[PRAVAH] Day phase starts at ") + formatTimeSec((uint32_t)dayStartMin * 60UL));
    return;
  }

  if (u.startsWith("E")) {
    String value = u.substring(1);
    value.trim();
    uint16_t mins = parseHHMM(value);
    if (mins == 65535 || mins <= dayStartMin) {
      Serial.println("Invalid day end. It must be HHMM and after day start.");
      return;
    }
    dayEndMin = mins;
    saveSchedule();
    webScheduleReady = false;
    Serial.println(String("[PRAVAH] Day phase ends at ") + formatTimeSec((uint32_t)dayEndMin * 60UL));
    return;
  }

  Serial.println("Unknown command. Use H for help.");
}

void readSerialCommands() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() > 0) {
        processCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      if (cmdBuffer.length() < 32) cmdBuffer += c;
    }
  }
}

// ----------------------------- Setup / loop -----------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n\nPRAVAH v3 starting...");

  const esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  const bool wokeFromTimer = (wakeCause == ESP_SLEEP_WAKEUP_TIMER);
  if (!wokeFromTimer) rtcClockValid = false;

  loadSchedule();
  loadBaseline();

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(20);

  awakeBaseMillis = millis();
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledReady = true;
    oledBrand();
  } else {
    Serial.println("[PRAVAH] OLED init failed.");
  }

  mpuReady = mpuBegin();
  logLine(String("MPU6050: ") + (mpuReady ? "OK" : "ERROR"));

  audioReady = audioI2SBegin();
  logLine(String("Audio I2S/ADC: ") + (audioReady ? "OK" : "ERROR"));

  // A timer wake means RTC logical time can be trusted from rtcScheduledTimeSec.
  // A cold boot/reset invalidates wall-clock time because this BOM has no RTC.
  if (wokeFromTimer && rtcClockValid) {
    Serial.println(String("[PRAVAH] Timer wake at ") + formatTimeSec(rtcScheduledTimeSec));
  }

  if (!rtcClockValid) {
    // Show only the brand on the OLED. Standalone users get the command hint over Serial.
    oledBrand();
    Serial.println("[PRAVAH] Set the clock after power-up: T HHMMSS (e.g. T 183045); T HHMM also works.");
    printHelp();
    delay(5000);
    oledOff();
    return;
  }

  if (!baseline.valid) {
    autoCalibrationPending = true;
  }

  printStatus();
}

bool scheduledTargetReached(uint32_t nowSec, uint32_t targetSec) {
  // Distinguish a target that is just past from one almost a full day in the future.
  return forwardSeconds(targetSec, nowSec) < (SECONDS_PER_DAY / 2UL);
}

void loop() {
  readSerialCommands();

  // If the desktop app closes unexpectedly or USB is removed, fall back to the
  // standalone schedule instead of staying fully awake indefinitely.
  if (webControlEnabled && !cycleRunning &&
      (uint32_t)(millis() - webLastCommandMillis) > WEB_CONTROL_TIMEOUT_MS) {
    webControlEnabled = false;
    webReleasePending = rtcClockValid;
    Serial.println("Web control: OFF");
    logLine("[PRAVAH] Web control timed out; returning to standalone scheduling.");
  }

  if (webReleasePending && !cycleRunning && rtcClockValid) {
    webReleasePending = false;
    standaloneSchedulePending = false;
    if (baseline.valid) {
      sleepUntilNextScheduledSlot();
      return;
    }
    // Without a baseline, stay awake long enough for a calibration attempt rather than scheduling readings.
  }

  if (autoCalibrationPending && rtcClockValid && !baseline.valid && !cycleRunning) {
    autoCalibrationPending = false;
    maybeRunAutomaticCalibration();
  }

  if (standaloneSchedulePending && !webControlEnabled && rtcClockValid && baseline.valid && !cycleRunning) {
    standaloneSchedulePending = false;
    sleepUntilNextScheduledSlot();
    return;
  }

  // While the desktop app is connected over USB, keep awake and execute scheduled
  // cycles from the same schedule instead of entering deep sleep.
  if (webControlEnabled && rtcClockValid && baseline.valid && audioReady && !cycleRunning) {
    const uint32_t nowSec = getLogicalNowSec();
    if (!webScheduleReady) {
      webNextScheduledTargetSec = nextScheduledTargetSec(nowSec);
      webScheduleReady = true;
      logLine(String("[PRAVAH] Web schedule ready; next measurement at ") + formatTimeSec(webNextScheduledTargetSec));
    } else if (scheduledTargetReached(nowSec, webNextScheduledTargetSec)) {
      automaticMeasurementCycle();
      // Recalculate after the cycle because ten-minute confirmation can span a scheduled slot.
      webNextScheduledTargetSec = nextScheduledTargetSec(getLogicalNowSec());
      logLine(String("[PRAVAH] Web schedule updated; next measurement at ") + formatTimeSec(webNextScheduledTargetSec));
    }
  }

  // After a timer wake, execute the scheduled cycle once.
  static bool initializedLoop = false;
  if (!initializedLoop) {
    initializedLoop = true;
    if (!webControlEnabled && rtcClockValid && baseline.valid && !cycleRunning) {
      if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        automaticMeasurementCycle();
      }
    }
  }

  delay(10);
}
