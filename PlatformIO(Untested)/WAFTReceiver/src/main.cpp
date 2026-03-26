#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_bt.h>

WiFiUDP udp;

namespace {

constexpr char kWifiSsid[] = "WAFTLink";
constexpr char kWifiPassword[] = "waftlink123";
constexpr uint8_t kWifiChannel = 6;
const IPAddress kApIp(192, 168, 4, 1);
const IPAddress kApGateway(192, 168, 4, 1);
const IPAddress kApSubnet(255, 255, 255, 0);
constexpr uint16_t kUdpPort = 3333;

constexpr uint8_t kProtocolVersion = 1;
constexpr uint16_t kMaxUdpPayloadBytes = 1460;
constexpr uint8_t kMaxSamplesPerPacket = 145;
constexpr uint32_t kStatsIntervalMs = 5000;
constexpr uint32_t kDebugEveryNBatches = 10;
constexpr uint32_t kNominalSampleSpacingMs = 4;
constexpr uint32_t kLargeGapThresholdMs = 5;
constexpr float kAdcReferenceVolts = 3.3f;
constexpr uint8_t kI2cSlaveAddress = 0x42;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint8_t kI2cFrameVersion = 1;
constexpr uint8_t kI2cSamplesPerFrame = 2;
constexpr size_t kI2cQueueCapacity = 64;

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

struct __attribute__((packed)) I2cSampleRecord {
  uint16_t dt_ms;
  int32_t ch1_raw;
  int32_t ch2_raw;
};

struct __attribute__((packed)) I2cSampleFrame {
  uint8_t version;
  uint8_t sample_count;
  uint16_t sequence;
  I2cSampleRecord samples[kI2cSamplesPerFrame];
};

uint8_t packetBuffer[kMaxUdpPayloadBytes] = {0};
I2cSampleRecord i2cSampleQueue[kI2cQueueCapacity] = {};
portMUX_TYPE i2cQueueMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t batchesReceived = 0;
uint32_t totalSamples = 0;
uint32_t malformedPackets = 0;
uint32_t missedBatches = 0;
uint32_t outOfOrderPackets = 0;
uint32_t inferredFrameGaps = 0;

bool haveLastSequence = false;
uint32_t lastSequence = 0;

bool haveLastSampleTime = false;
uint32_t lastSampleRxMs = 0;
uint16_t lastSampleDtMs = 0;
float lastCh1Voltage = 0.0f;
float lastCh2Voltage = 0.0f;
uint32_t lastPacketRxMs = 0;
uint32_t lastStatsMs = 0;
size_t i2cQueueHead = 0;
size_t i2cQueueTail = 0;
size_t i2cQueueCount = 0;
size_t i2cQueueHighWaterMark = 0;
uint16_t i2cFrameSequence = 0;
uint32_t i2cFramesServed = 0;
uint32_t i2cSamplesServed = 0;
uint32_t i2cSamplesDropped = 0;

float rawToVoltage(int32_t rawValue) {
  return (static_cast<float>(rawValue) * kAdcReferenceVolts) / 8388608.0f;
}

void enqueueI2cSample(const SampleRecord &sample) {
  portENTER_CRITICAL(&i2cQueueMux);

  if (i2cQueueCount == kI2cQueueCapacity) {
    i2cQueueTail = (i2cQueueTail + 1U) % kI2cQueueCapacity;
    i2cQueueCount--;
    i2cSamplesDropped++;
  }

  i2cSampleQueue[i2cQueueHead] = {sample.dt_ms, sample.ch1_raw, sample.ch2_raw};
  i2cQueueHead = (i2cQueueHead + 1U) % kI2cQueueCapacity;
  i2cQueueCount++;

  if (i2cQueueCount > i2cQueueHighWaterMark) {
    i2cQueueHighWaterMark = i2cQueueCount;
  }

  portEXIT_CRITICAL(&i2cQueueMux);
}

void onI2cRequest() {
  I2cSampleFrame frame = {};
  frame.version = kI2cFrameVersion;

  portENTER_CRITICAL(&i2cQueueMux);

  frame.sequence = i2cFrameSequence++;
  while (frame.sample_count < kI2cSamplesPerFrame && i2cQueueCount > 0U) {
    frame.samples[frame.sample_count] = i2cSampleQueue[i2cQueueTail];
    i2cQueueTail = (i2cQueueTail + 1U) % kI2cQueueCapacity;
    i2cQueueCount--;
    frame.sample_count++;
  }
  i2cFramesServed++;
  i2cSamplesServed += frame.sample_count;

  portEXIT_CRITICAL(&i2cQueueMux);

  Wire.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
}

bool validatePacketLayout(const BatchHeader &header, size_t packetSize) {
  if (header.version != kProtocolVersion) {
    return false;
  }

  if (header.sample_count == 0 || header.sample_count > kMaxSamplesPerPacket) {
    return false;
  }

  const size_t expectedSize = sizeof(BatchHeader) +
                              (static_cast<size_t>(header.sample_count) * sizeof(SampleRecord));
  return expectedSize == packetSize;
}

void processPacket(const uint8_t *packetData, size_t packetSize, uint32_t packetRxMs) {
  if (packetSize < sizeof(BatchHeader)) {
    malformedPackets++;
    return;
  }

  const BatchHeader *header = reinterpret_cast<const BatchHeader *>(packetData);
  if (!validatePacketLayout(*header, packetSize)) {
    malformedPackets++;
    return;
  }

  const SampleRecord *samples =
      reinterpret_cast<const SampleRecord *>(packetData + sizeof(BatchHeader));

  for (uint8_t i = 1; i < header->sample_count; ++i) {
    if (samples[i].dt_ms < samples[i - 1].dt_ms) {
      malformedPackets++;
      return;
    }
  }

  if (haveLastSequence) {
    const uint32_t expectedSequence = lastSequence + 1;
    if (header->sequence > expectedSequence) {
      missedBatches += (header->sequence - expectedSequence);
    } else if (header->sequence < expectedSequence) {
      outOfOrderPackets++;
      return;
    }
  }

  haveLastSequence = true;
  lastSequence = header->sequence;
  lastPacketRxMs = packetRxMs;
  batchesReceived++;
  totalSamples += header->sample_count;

  const uint16_t lastDtMs = samples[header->sample_count - 1].dt_ms;

  for (uint8_t i = 0; i < header->sample_count; ++i) {
    const SampleRecord &sample = samples[i];
    const uint32_t sampleRxMs = packetRxMs - (lastDtMs - sample.dt_ms);

    if (haveLastSampleTime && sampleRxMs > lastSampleRxMs) {
      const uint32_t deltaMs = sampleRxMs - lastSampleRxMs;
      if (deltaMs > kLargeGapThresholdMs) {
        const uint32_t intervals = (deltaMs + (kNominalSampleSpacingMs / 2)) / kNominalSampleSpacingMs;
        if (intervals > 1U) {
          inferredFrameGaps += (intervals - 1U);
        }
      }
    }

    haveLastSampleTime = true;
    lastSampleRxMs = sampleRxMs;
    lastSampleDtMs = sample.dt_ms;
    lastCh1Voltage = rawToVoltage(sample.ch1_raw);
    lastCh2Voltage = rawToVoltage(sample.ch2_raw);
    enqueueI2cSample(sample);
  }

  if ((batchesReceived % kDebugEveryNBatches) == 0U) {
    Serial.print("Batch ");
    Serial.print(header->sequence);
    Serial.print(" samples=");
    Serial.print(header->sample_count);
    Serial.print(" span=");
    Serial.print(lastDtMs);
    Serial.print("ms last_rx=");
    Serial.print(lastSampleRxMs);
    Serial.print(" ch1=");
    Serial.print(lastCh1Voltage, 4);
    Serial.print("V ch2=");
    Serial.print(lastCh2Voltage, 4);
    Serial.println("V");
  }
}

void printStats() {
  const uint32_t nowMs = millis();
  if ((nowMs - lastStatsMs) < kStatsIntervalMs) {
    return;
  }
  lastStatsMs = nowMs;

  size_t queuedSamples = 0;
  size_t queueHighWaterMark = 0;
  uint32_t samplesDropped = 0;
  uint32_t framesServed = 0;
  uint32_t samplesServed = 0;

  portENTER_CRITICAL(&i2cQueueMux);
  queuedSamples = i2cQueueCount;
  queueHighWaterMark = i2cQueueHighWaterMark;
  samplesDropped = i2cSamplesDropped;
  framesServed = i2cFramesServed;
  samplesServed = i2cSamplesServed;
  portEXIT_CRITICAL(&i2cQueueMux);

  Serial.print("Batches=");
  Serial.print(batchesReceived);
  Serial.print(" samples=");
  Serial.print(totalSamples);
  Serial.print(" missed_batches=");
  Serial.print(missedBatches);
  Serial.print(" malformed=");
  Serial.print(malformedPackets);
  Serial.print(" out_of_order=");
  Serial.print(outOfOrderPackets);
  Serial.print(" inferred_frame_gaps=");
  Serial.print(inferredFrameGaps);
  Serial.print(" i2c_queued=");
  Serial.print(queuedSamples);
  Serial.print(" i2c_q_high=");
  Serial.print(queueHighWaterMark);
  Serial.print(" i2c_frames=");
  Serial.print(framesServed);
  Serial.print(" i2c_samples=");
  Serial.print(samplesServed);
  Serial.print(" i2c_dropped=");
  Serial.print(samplesDropped);
  Serial.print(" stations=");
  Serial.print(WiFi.softAPgetStationNum());

  if (haveLastSampleTime) {
    Serial.print(" last_packet_rx=");
    Serial.print(lastPacketRxMs);
    Serial.print("ms last_dt=");
    Serial.print(lastSampleDtMs);
    Serial.print("ms ch1=");
    Serial.print(lastCh1Voltage, 4);
    Serial.print("V ch2=");
    Serial.print(lastCh2Voltage, 4);
    Serial.print("V");
  }

  Serial.println();
}

}  // namespace

void setup() {
  setCpuFrequencyMhz(160);

  Serial.begin(115200);
  delay(500);
  Serial.println("=================================");
  Serial.println("WAFT UDP Receiver");
  Serial.println("SoftAP + UDP batch listener");
  Serial.println("=================================");

  Wire.onRequest(onI2cRequest);
  if (!Wire.begin(kI2cSlaveAddress, SDA, SCL, kI2cClockHz)) {
    Serial.println("ERROR: Failed to start I2C slave");
    while (true) {
      delay(1000);
    }
  }

  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  if (!WiFi.softAPConfig(kApIp, kApGateway, kApSubnet)) {
    Serial.println("ERROR: Failed to configure SoftAP IP");
    while (true) {
      delay(1000);
    }
  }

  if (!WiFi.softAP(kWifiSsid, kWifiPassword, kWifiChannel, 0, 1)) {
    Serial.println("ERROR: Failed to start SoftAP");
    while (true) {
      delay(1000);
    }
  }

  WiFi.setTxPower(WIFI_POWER_2dBm);

  if (!udp.begin(kUdpPort)) {
    Serial.println("ERROR: Failed to bind UDP port");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("SoftAP IP=");
  Serial.println(WiFi.softAPIP());
  Serial.print("Listening on UDP port ");
  Serial.println(kUdpPort);
  Serial.print("I2C slave address=0x");
  Serial.print(kI2cSlaveAddress, HEX);
  Serial.print(" SDA=");
  Serial.print(SDA);
  Serial.print(" SCL=");
  Serial.println(SCL);
}

void loop() {
  const int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    const uint32_t packetRxMs = millis();
    if (packetSize > static_cast<int>(sizeof(packetBuffer))) {
      udp.read(packetBuffer, sizeof(packetBuffer));
      malformedPackets++;
    } else {
      const int bytesRead = udp.read(packetBuffer, packetSize);
      if (bytesRead == packetSize) {
        processPacket(packetBuffer, static_cast<size_t>(bytesRead), packetRxMs);
      } else {
        malformedPackets++;
      }
    }
  }

  printStats();
}
