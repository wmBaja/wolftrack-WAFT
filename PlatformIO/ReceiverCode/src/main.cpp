/*
  ESP-NOW Receiver for ADS122C04 Dual Differential Measurements
  Optimized to reduce packet loss at 5ms intervals (200 Hz)
*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

/* ================= PACKET STRUCT ================= */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  int32_t ch1_raw;
  int32_t ch2_raw;
  float ch1_voltage;
  float ch2_voltage;
} adc_packet_t;

adc_packet_t incoming;

// Statistics tracking
uint32_t last_packet_time = 0;
uint32_t packet_count = 0;
uint32_t missed_packets = 0;
bool first_packet = true;

// Print throttling - only print every N packets to reduce serial blocking
const uint8_t PRINT_EVERY_N = 10;  // Print every 10th packet (50ms intervals)
uint8_t print_counter = 0;

/* ================= ESP-NOW CALLBACK ================= */
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(adc_packet_t)) {
    return;  // Silently ignore malformed packets
  }
  
  memcpy(&incoming, data, sizeof(incoming));
  packet_count++;
  
  // Calculate time since last packet
  uint32_t delta_ms = 0;
  if (!first_packet) {
    delta_ms = incoming.timestamp_ms - last_packet_time;
    
    // Detect missed packets (allowing for 1ms jitter)
    if (delta_ms > 6) {
      uint32_t expected_packets = delta_ms / 5;
      missed_packets += (expected_packets - 1);
    }
  } else {
    first_packet = false;
  }
  last_packet_time = incoming.timestamp_ms;
  
  // Only print every Nth packet to reduce serial blocking
  print_counter++;
  if (print_counter >= PRINT_EVERY_N) {
    print_counter = 0;
    
    Serial.print("Packets: ");
    Serial.print(packet_count);
    Serial.print(" | Missed: ");
    Serial.print(missed_packets);
    Serial.print(" | Δt=");
    Serial.print(delta_ms);
    Serial.print("ms | CH1: ");
    Serial.print(incoming.ch1_voltage, 4);
    Serial.print("V | CH2: ");
    Serial.print(incoming.ch2_voltage, 4);
    Serial.println("V");
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=================================");
  Serial.println("ESP-NOW Receiver - Optimized");
  Serial.println("Packet Rate: 200 Hz (5ms interval)");
  Serial.println("=================================");
  
  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }
  
  // Note: Advanced WiFi functions removed for compatibility
  // Basic ESP-NOW should still work well
  
  esp_now_register_recv_cb(onReceive);
  
  Serial.println("Waiting for packets...");
  Serial.println("(Printing every 10th packet to reduce blocking)");
  Serial.println("=================================\n");
}

/* ================= LOOP ================= */
void loop() {
  // Optional: Print statistics every 5 seconds
  static uint32_t last_stats_print = 0;
  if (millis() - last_stats_print > 5000) {
    last_stats_print = millis();
    float success_rate = 100.0 * packet_count / (packet_count + missed_packets);
    Serial.println();
    Serial.print(">>> 5s Stats: Received=");
    Serial.print(packet_count);
    Serial.print(" | Missed=");
    Serial.print(missed_packets);
    Serial.print(" | Success Rate=");
    Serial.print(success_rate, 1);
    Serial.println("%");
    Serial.println();
  }
}