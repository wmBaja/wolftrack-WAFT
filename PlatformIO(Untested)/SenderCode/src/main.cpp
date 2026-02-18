/*
  ADS122C04 Dual Differential Measurement - ESP32
  ESP-NOW Sender (Fixed 5ms Interval)
  
  Features:
  - Continuous conversion mode
  - Turbo mode (2000 SPS)
  - Precise 5ms transmission interval
  - Microsecond-level timing control
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SparkFun_ADS122C04_ADC_Arduino_Library.h>

SFE_ADS122C04 mySensor;

/* ================= ESP-NOW CONFIG ================= */
// Receiver ESP32 MAC address
uint8_t receiverMAC[] = {0xEC, 0x62, 0x60, 0x76, 0xB0, 0xD4};

typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  int32_t ch1_raw;
  int32_t ch2_raw;
  float ch1_voltage;
  float ch2_voltage;
} adc_packet_t;

adc_packet_t packet;

/* ================= TIMING CONTROL ================= */
const uint32_t INTERVAL_US = 5000;  // 5ms = 5000 microseconds
uint32_t nextTransmitTime = 0;

/* ================= ESP-NOW SEND STATUS ================= */
void onSend(const uint8_t *mac, esp_now_send_status_t status) {
  // Silent operation for speed
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=================================");
  Serial.println("ADS122C04 ESP-NOW Sender");
  Serial.println("Fixed 5ms Transmission Interval");
  Serial.println("=================================");
  
  /* ---- I2C Init ---- */
  Wire.begin();
  Wire.setClock(400000);
  
  if (!mySensor.begin(0x40)) {
    Serial.println("ERROR: ADS122C04 not detected!");
    while (1);
  }
  Serial.println("✓ ADS122C04 detected");
  
  /* ---- ADC Configuration ---- */
  mySensor.configureADCmode(ADS122C04_RAW_MODE);
  mySensor.setGain(ADS122C04_GAIN_1);
  mySensor.setDataRate(ADS122C04_DATA_RATE_1000SPS);
  mySensor.setVoltageReference(ADS122C04_VREF_EXT_REF_PINS);
  mySensor.setConversionMode(ADS122C04_CONVERSION_MODE_CONTINUOUS);
  mySensor.setOperatingMode(ADS122C04_OP_MODE_TURBO);
  Serial.println("✓ ADC configured (2000 SPS, Turbo, Continuous)");
  
  /* ---- ESP-NOW Init ---- */
  WiFi.mode(WIFI_STA);
  Serial.print("✓ MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed!");
    while (1);
  }
  Serial.println("✓ ESP-NOW initialized");
  
  // Note: Advanced WiFi functions removed for compatibility
  // Basic ESP-NOW should still work well
  
  esp_now_register_send_cb(onSend);
  
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERROR: Failed to add peer!");
    while (1);
  }
  
  Serial.print("✓ Peer added: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", receiverMAC[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  Serial.println("=================================");
  Serial.println("Starting 5ms interval transmission...");
  Serial.println("=================================\n");
  
  delay(1000);
  
  // Initialize timing reference
  nextTransmitTime = micros() + INTERVAL_US;
}

/* ================= MAIN LOOP ================= */
void loop() {
  // Wait until it's time to transmit
  while (micros() < nextTransmitTime) {
    // Tight loop for precise timing
  }
  
  // Calculate next transmission time FIRST to maintain consistency
  nextTransmitTime += INTERVAL_US;
  
  uint32_t rawData;
  int32_t signedData;
  
  packet.timestamp_ms = millis();
  
  /* ---- Channel 1: AIN3 - AIN2 ---- */
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN3_AIN2);
  mySensor.start();
  delayMicroseconds(500);
  rawData = mySensor.readADC();
  signedData = (int32_t)rawData;
  if (signedData & 0x800000) signedData |= 0xFF000000;
  packet.ch1_raw = signedData;
  packet.ch1_voltage = (signedData * 3.3) / 8388608.0;
  
  /* ---- Channel 2: AIN1 - AIN0 ---- */
  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN1_AIN0);
  mySensor.start();
  delayMicroseconds(500);
  rawData = mySensor.readADC();
  signedData = (int32_t)rawData;
  if (signedData & 0x800000) signedData |= 0xFF000000;
  packet.ch2_raw = signedData;
  packet.ch2_voltage = (signedData * 3.3) / 8388608.0;
  
  /* ---- ESP-NOW Send ---- */
  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
}