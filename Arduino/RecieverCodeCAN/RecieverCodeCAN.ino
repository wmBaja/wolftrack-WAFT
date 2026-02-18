/*
  ESP-NOW Receiver → TWAI (CAN Bus) Transmitter
  Receives ADS122C04 dual differential data via ESP-NOW
  and forwards it as CAN frames via TWAI + SN65HVD230 transceiver
*/
#include <WiFi.h>
#include <esp_now.h>
#include "driver/twai.h"

/* ================= PIN DEFINITIONS ================= */
#define CAN_TX_PIN  GPIO_NUM_17   // TX2 — left side
#define CAN_RX_PIN  GPIO_NUM_16   // RX2 — left side

/* ================= PACKET STRUCT ================= */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  int32_t  ch1_raw;
  int32_t  ch2_raw;
  float    ch1_voltage;
  float    ch2_voltage;
} adc_packet_t;

adc_packet_t incoming;

/* ================= CAN FRAME IDs ================= */
// Split the data across two 8-byte CAN frames
#define CAN_ID_CH1   0x100   // CH1 voltage + raw
#define CAN_ID_CH2   0x101   // CH2 voltage + raw

/* ================= STATISTICS ================= */
uint32_t last_packet_time  = 0;
uint32_t packet_count      = 0;
uint32_t missed_packets    = 0;
uint32_t can_tx_errors     = 0;
bool     first_packet      = true;

const uint8_t PRINT_EVERY_N = 10;
uint8_t       print_counter = 0;

/* ================= HELPERS ================= */
// Pack a float into 4 bytes (little-endian)
void pack_float(uint8_t *buf, float val) {
  memcpy(buf, &val, 4);
}

// Pack an int32 into 4 bytes (little-endian)
void pack_int32(uint8_t *buf, int32_t val) {
  memcpy(buf, &val, 4);
}

/* ================= TWAI INIT ================= */
void twai_setup() {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

  // 500 kbps — common, reliable rate. Change to TWAI_TIMING_CONFIG_250KBITS()
  // or TWAI_TIMING_CONFIG_1MBITS() to match your bus.
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI driver install failed");
    while (1);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    while (1);
  }
  Serial.println("TWAI (CAN) started at 500 kbps");
}

/* ================= SEND CAN FRAMES ================= */
void send_can_frames(const adc_packet_t &pkt) {
  // --- Frame 1: CH1 voltage (4 bytes) + CH1 raw (4 bytes) ---
  twai_message_t frame1;
  frame1.identifier       = CAN_ID_CH1;
  frame1.extd             = 0;          // Standard 11-bit ID
  frame1.rtr              = 0;
  frame1.data_length_code = 8;
  pack_float( &frame1.data[0], pkt.ch1_voltage);
  pack_int32( &frame1.data[4], pkt.ch1_raw);

  if (twai_transmit(&frame1, pdMS_TO_TICKS(2)) != ESP_OK) {
    can_tx_errors++;
  }

  // --- Frame 2: CH2 voltage (4 bytes) + CH2 raw (4 bytes) ---
  twai_message_t frame2;
  frame2.identifier       = CAN_ID_CH2;
  frame2.extd             = 0;
  frame2.rtr              = 0;
  frame2.data_length_code = 8;
  pack_float( &frame2.data[0], pkt.ch2_voltage);
  pack_int32( &frame2.data[4], pkt.ch2_raw);

  if (twai_transmit(&frame2, pdMS_TO_TICKS(2)) != ESP_OK) {
    can_tx_errors++;
  }
}

/* ================= ESP-NOW CALLBACK ================= */
void onReceive(const esp_now_recv_info *info,
               const uint8_t *data,
               int len) {
  if (len != sizeof(adc_packet_t)) return;

  memcpy(&incoming, data, sizeof(incoming));
  packet_count++;

  uint32_t delta_ms = 0;
  if (!first_packet) {
    delta_ms = incoming.timestamp_ms - last_packet_time;
    if (delta_ms > 6) {
      missed_packets += (delta_ms / 5) - 1;
    }
  } else {
    first_packet = false;
  }
  last_packet_time = incoming.timestamp_ms;

  // Forward data onto CAN bus immediately
  send_can_frames(incoming);

  // Throttled serial debug
  if (++print_counter >= PRINT_EVERY_N) {
    print_counter = 0;
    Serial.print("Pkts: ");    Serial.print(packet_count);
    Serial.print(" | Miss: "); Serial.print(missed_packets);
    Serial.print(" | CAN Err: "); Serial.print(can_tx_errors);
    Serial.print(" | Δt=");    Serial.print(delta_ms);
    Serial.print("ms | CH1: ");Serial.print(incoming.ch1_voltage, 4);
    Serial.print("V | CH2: "); Serial.print(incoming.ch2_voltage, 4);
    Serial.println("V");
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("ESP-NOW → TWAI CAN Bridge");
  Serial.println("=================================");

  // TWAI must be started before WiFi on some IDF versions,
  // but ESP-NOW needs WiFi — start TWAI first, it uses separate hardware.
  twai_setup();

  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }
  esp_now_register_recv_cb(onReceive);

  Serial.println("Waiting for ESP-NOW packets...");
  Serial.println("=================================\n");
}

/* ================= LOOP ================= */
void loop() {
  static uint32_t last_stats = 0;
  if (millis() - last_stats > 5000) {
    last_stats = millis();
    float success = (packet_count + missed_packets) > 0
                    ? 100.0f * packet_count / (packet_count + missed_packets)
                    : 0;
    Serial.println();
    Serial.print(">>> 5s Stats | RX: "); Serial.print(packet_count);
    Serial.print(" | Miss: ");           Serial.print(missed_packets);
    Serial.print(" | CAN Err: ");        Serial.print(can_tx_errors);
    Serial.print(" | Success: ");        Serial.print(success, 1);
    Serial.println("%\n");
  }
}