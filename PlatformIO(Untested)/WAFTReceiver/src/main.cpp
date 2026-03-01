#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 


/* ================= PACKET STRUCT ================= */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  int32_t  ch1_raw;
  int32_t  ch2_raw;
  float    ch1_voltage;
  float    ch2_voltage;
} adc_packet_t;

adc_packet_t incoming;


/* ================= STATISTICS ================= */
uint32_t last_packet_time  = 0;
uint32_t packet_count      = 0;
uint32_t missed_packets    = 0;
uint32_t can_tx_errors     = 0;
bool     first_packet      = true;

const uint8_t PRINT_EVERY_N = 100;
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


void onReceive(const uint8_t *mac_addr,
               const uint8_t *data,
               int data_len);


void setup() {
  // put your setup code here, to run once:
  setCpuFrequencyMhz(160); // underclock to 80 MHz

  Serial.begin(115200);
  delay(500);
  Serial.println("WAFT Receiver");

  Wire.begin();
  Wire.setClock(400000);

  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(8); // ~2 dBm — valid range is 8–84 (units of 0.25 dBm)
  btStop(); // Turn off bluetooth

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed!");
    while (1);
  }
  Serial.println("✓ ESP-NOW initialized");

  esp_now_register_recv_cb(onReceive);
}

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

void onReceive(const uint8_t *mac_addr,
               const uint8_t *data,
               int data_len) {
  if (data_len != sizeof(adc_packet_t)) return;

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

  // TODO: I2C SEND HERE

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
