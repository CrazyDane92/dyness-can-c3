#include <Arduino.h>
#include "esp_task_wdt.h"

HardwareSerial &UART = Serial1;  // HW UART (TX=GPIO21, RX=GPIO20 on ESP32-C3)

void setup() {
  // Give USB stack time to come up
  delay(400);
  Serial.begin(115200);     // USB CDC
  delay(400);

  // Start HW UART too (fallback if USB is stubborn)
  UART.begin(115200, SERIAL_8N1, 20, 21); // RX=20, TX=21

  // Disable task watchdog to avoid surprise resets
  esp_task_wdt_deinit();

  // Hello banners on both outputs
  Serial.println("\n=== USB-GOLDEN TEST (ESP32-C3) ===");
  UART.println("\n=== USB-GOLDEN TEST (ESP32-C3) ===");
  Serial.println("USB CDC should be visible as 'USB Serial Device (COMx)'.");
  UART.println("USB CDC should be visible as 'USB Serial Device (COMx)'.");
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 1000) {
    t = millis();
    Serial.printf("HELLO (USB)  %lu ms\n", (unsigned long)t);
    UART.printf(  "HELLO (UART) %lu ms\n", (unsigned long)t);
  }
}
