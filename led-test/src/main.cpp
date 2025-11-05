#include <Arduino.h>

#define LED_PIN 8   // Your LED pin

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("BOOT OK");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
