#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SparkFun_ADS122C04_ADC_Arduino_Library.h>
#include <esp_bt.h>
#include <esp_timer.h>

SFE_ADS122C04 mySensor;
WiFiUDP udp;

namespace {

constexpr char kWifiSsid[] = "WAFTLink";
constexpr char kWifiPassword[] = "waftlink123";
constexpr uint8_t kWifiChannel = 6;
const IPAddress kReceiverIp(192, 168, 4, 1);
constexpr uint16_t kUdpPort = 3333;

constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kBatchSamples = 50;
constexpr uint16_t kMaxSamplesPerPacket = 145;
constexpr uint16_t kMaxUdpPayloadBytes = 1460;

constexpr uint32_t kSamplePeriodUs = 4000;
constexpr uint32_t kConversionTimeoutUs = 2000;
constexpr uint32_t kWifiRetryIntervalMs = 2000;
constexpr uint32_t kStatsIntervalMs = 5000;
constexpr float kAdcReferenceVolts = 3.3f;
constexpr uint32_t kI2cClockHz = 400000;

struct __attribute__((packed)) BatchHeader {
  uint8_t version;
  uint8_t sample_count;
  uint16_t flags;
  uint32_t sequence;
};

struct __attribute__((packed)) SampleRecord {
  uint16_t dt_ms;
  int32_t ch1_raw;
  int32_t ch2_raw;
};

struct __attribute__((packed)) SampleBatch {
  BatchHeader header;
  SampleRecord samples[kBatchSamples];
};

static_assert(sizeof(BatchHeader) + (sizeof(SampleRecord) * kBatchSamples) <= kMaxUdpPayloadBytes,
              "Configured batch exceeds UDP payload budget");
static_assert(kBatchSamples <= kMaxSamplesPerPacket,
              "Configured batch exceeds supported sample count");

SampleBatch batch = {};
uint8_t batchCount = 0;
uint32_t nextSequence = 0;
uint64_t batchStartUs = 0;
uint64_t nextFrameUs = 0;

uint32_t sentBatches = 0;
uint32_t droppedBatches = 0;
uint32_t adcTimeouts = 0;
uint32_t lateFrameRecoveries = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t lastStatsMs = 0;
bool wifiWasConnected = false;

int32_t signExtend24(uint32_t rawValue) {
  int32_t signedValue = static_cast<int32_t>(rawValue & 0x00FFFFFFUL);
  if ((signedValue & 0x00800000L) != 0) {
    signedValue |= static_cast<int32_t>(0xFF000000UL);
  }
  return signedValue;
}

float rawToVoltage(int32_t rawValue) {
  return (static_cast<float>(rawValue) * kAdcReferenceVolts) / 8388608.0f;
}

bool waitForDataReady(uint32_t timeoutUs) {
  const uint64_t deadlineUs = esp_timer_get_time() + timeoutUs;
  while (esp_timer_get_time() < deadlineUs) {
    if (mySensor.checkDataReady()) {
      return true;
    }
    delayMicroseconds(50);
  }
  return mySensor.checkDataReady();
}

bool readChannel(uint8_t muxConfig, int32_t &rawOut) {
  if (!mySensor.setInputMultiplexer(muxConfig)) {
    return false;
  }
  if (!mySensor.start()) {
    return false;
  }
  if (!waitForDataReady(kConversionTimeoutUs)) {
    return false;
  }

  rawOut = signExtend24(mySensor.readADC());
  return true;
}

void beginWifiConnection() {
  WiFi.begin(kWifiSsid, kWifiPassword, kWifiChannel);
  WiFi.setTxPower(WIFI_POWER_2dBm);
  lastWifiAttemptMs = millis();
}

void maintainWifiConnection() {
  const bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print("WiFi connected, IP=");
      Serial.println(WiFi.localIP());
    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("WiFi disconnected");
  }

  const uint32_t nowMs = millis();
  if ((nowMs - lastWifiAttemptMs) >= kWifiRetryIntervalMs) {
    Serial.println("Retrying WiFi connection...");
    beginWifiConnection();
  }
}

void flushBatch() {
  if (batchCount == 0) {
    return;
  }

  batch.header.version = kProtocolVersion;
  batch.header.sample_count = batchCount;
  batch.header.flags = 0;
  batch.header.sequence = nextSequence++;

  const size_t packetSize = sizeof(BatchHeader) + (static_cast<size_t>(batchCount) * sizeof(SampleRecord));
  bool sent = false;

  if (WiFi.status() == WL_CONNECTED && udp.beginPacket(kReceiverIp, kUdpPort)) {
    const size_t written = udp.write(reinterpret_cast<const uint8_t *>(&batch), packetSize);
    sent = (written == packetSize) && (udp.endPacket() == 1);
  }

  if (sent) {
    sentBatches++;
  } else {
    droppedBatches++;
  }

  batchCount = 0;
  batchStartUs = 0;
}

bool captureSampleFrame() {
  int32_t ch1Raw = 0;
  int32_t ch2Raw = 0;

  const uint64_t frameStartUs = esp_timer_get_time();

  if (!readChannel(ADS122C04_MUX_AIN3_AIN2, ch1Raw) ||
      !readChannel(ADS122C04_MUX_AIN1_AIN0, ch2Raw)) {
    adcTimeouts++;
    return false;
  }

  if (batchCount == 0) {
    batchStartUs = frameStartUs;
  }

  SampleRecord &sample = batch.samples[batchCount];
  sample.dt_ms = static_cast<uint16_t>(((frameStartUs - batchStartUs) + 500ULL) / 1000ULL);
  sample.ch1_raw = ch1Raw;
  sample.ch2_raw = ch2Raw;

  batchCount++;
  if (batchCount >= kBatchSamples) {
    flushBatch();
  }

  return true;
}

void printStats() {
  const uint32_t nowMs = millis();
  if ((nowMs - lastStatsMs) < kStatsIntervalMs) {
    return;
  }
  lastStatsMs = nowMs;

  Serial.print("Batches sent=");
  Serial.print(sentBatches);
  Serial.print(" dropped=");
  Serial.print(droppedBatches);
  Serial.print(" adc_timeouts=");
  Serial.print(adcTimeouts);
  Serial.print(" late_recoveries=");
  Serial.print(lateFrameRecoveries);
  Serial.print(" wifi=");
  Serial.print(WiFi.status() == WL_CONNECTED ? "up" : "down");

  if (batchCount > 0) {
    const SampleRecord &lastSample = batch.samples[batchCount - 1];
    Serial.print(" pending=");
    Serial.print(batchCount);
    Serial.print(" last_dt=");
    Serial.print(lastSample.dt_ms);
    Serial.print("ms ch1=");
    Serial.print(rawToVoltage(lastSample.ch1_raw), 4);
    Serial.print("V ch2=");
    Serial.print(rawToVoltage(lastSample.ch2_raw), 4);
    Serial.print("V");
  }

  Serial.println();
}

void configureSensor() {
  if (!mySensor.begin(0x40)) {
    Serial.println("ERROR: ADS122C04 not detected!");
    Serial.println("Check wiring and I2C address.");
    while (true) {
      delay(1000);
    }
  }

  mySensor.configureADCmode(ADS122C04_RAW_MODE);
  mySensor.setGain(ADS122C04_GAIN_1);
  mySensor.setDataRate(ADS122C04_DATA_RATE_1000SPS);
  mySensor.setVoltageReference(ADS122C04_VREF_EXT_REF_PINS);
  mySensor.setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT);
  mySensor.setOperatingMode(ADS122C04_OP_MODE_TURBO);
}

}  // namespace

void setup() {
  setCpuFrequencyMhz(160);

  Serial.begin(115200);
  delay(500);
  Serial.println("=================================");
  Serial.println("WAFT UDP Sender");
  Serial.println("250 SPS batched WiFi transport");
  Serial.println("=================================");

  Wire.begin();
  Wire.setClock(kI2cClockHz);
  configureSensor();

  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  beginWifiConnection();

  Serial.println("ADC configured: 1000 SPS, turbo, single-shot");
  Serial.println("WiFi connecting to WAFTLink...");
  nextFrameUs = esp_timer_get_time();
}

void loop() {
  maintainWifiConnection();
  printStats();

  const uint64_t nowUs = esp_timer_get_time();
  if (nowUs < nextFrameUs) {
    if ((nextFrameUs - nowUs) > 200U) {
      delayMicroseconds(100);
    }
    return;
  }

  if ((nowUs - nextFrameUs) > kSamplePeriodUs) {
    const uint64_t skippedFrames = (nowUs - nextFrameUs) / kSamplePeriodUs;
    lateFrameRecoveries += static_cast<uint32_t>(skippedFrames);
    nextFrameUs += skippedFrames * static_cast<uint64_t>(kSamplePeriodUs);
  }

  nextFrameUs += kSamplePeriodUs;
  captureSampleFrame();
}
