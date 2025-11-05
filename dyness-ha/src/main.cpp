#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// ESP32-C3 Super Mini + MCP2515 (HW-184)
// SPI pins: SCK=GPIO4, MISO=GPIO5, MOSI=GPIO6, CS=GPIO7
#define CAN_CS 7

MCP2515 mcp2515(CAN_CS);

static const char* bms_state_text(uint8_t s) {
  switch (s) {
    case 0: return "Idle";
    case 1: return "Standby";
    case 2: return "Precharge";
    case 3: return "Discharge";
    case 4: return "Charge";
    case 5: return "Sleep";
    case 6: return "Fault/Protect";
    default: return "Unknown";
  }
}

void setup() {
  // Force USB CDC serial and give it time to come up
  delay(300);
  Serial.begin(115200);
  delay(300);
  Serial.println("\nDyness CAN Decoder (ESP32-C3 + MCP2515 @ 500kbps, 8MHz)");

  // SPI & MCP2515 init (polling mode, no INT pin)
  SPI.begin(/*SCK*/4, /*MISO*/5, /*MOSI*/6, /*SS*/CAN_CS);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("MCP2515 OK, listening…");
}

void loop() {
  struct can_frame frame;
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    const bool ext = frame.can_id & CAN_EFF_FLAG;
    const uint32_t id = frame.can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK);

    // Print every frame
    Serial.printf("[%s] ID=0x%08lX len=%u data=", ext ? "EXT" : "STD", (unsigned long)id, frame.can_dlc);
    for (uint8_t i = 0; i < frame.can_dlc; i++) Serial.printf("%s%02X", i ? " " : "", frame.data[i]);
    Serial.println();

    // -------- Dyness decoding ----------
    // 0x356 (std): Voltage/Current/Power (as seen in your captures)
    if (!ext && id == 0x356 && frame.can_dlc >= 6) {
      uint16_t vraw = (uint16_t)(frame.data[0] | (frame.data[1] << 8)); // 0.01 V
      int16_t  iraw = (int16_t)(frame.data[2] | (frame.data[3] << 8));  // 0.1 A (signed)
      float V = vraw / 100.0f;
      float A = iraw / 10.0f;
      float W = V * A;
      Serial.printf("  → Voltage=%.2f V, Current=%.1f A, Power=%.0f W\n", V, A, W);
    }

    // 0x18AA0101 (ext): SOC, SOH, state, alarms (from your frames)
    if (ext && id == 0x18AA0101 && frame.can_dlc >= 8) {
      uint8_t soc = frame.data[4];
      uint8_t soh = frame.data[5];
      uint8_t st  = frame.data[6];
      uint8_t alm = frame.data[7];
      Serial.printf("  → SOC=%u%%, SOH=%u%%, State=%s (%u), Alarms=0x%02X\n",
                    soc, soh, bms_state_text(st), st, alm);
    }

    // 0x35E / 0x371 (std): ASCII chunks ("DYNESS-L", " BATTERY")
    if (!ext && (id == 0x35E || id == 0x371) && frame.can_dlc == 8) {
      char txt[9]; memcpy(txt, frame.data, 8); txt[8] = 0;
      Serial.printf("  → Text: \"%s\"\n", txt);
    }
  }

  // Modest polling to keep CPU cool
  delayMicroseconds(300);
}
