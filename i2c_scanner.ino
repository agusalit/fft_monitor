#include <Wire.h>

const int pairs[][2] = {
  {21, 22},  // default
  {22, 21},
  {18, 19},
  {19, 18},
  {17, 16},
  {16, 17},
  {25, 26},
  {26, 25},
  {4, 5},
  {5, 4},
  {14, 15},
  {15, 14},
  {13, 12},
  {12, 13}
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nMulti-pin I2C scanner starting...");
  delay(200);
}

void loop() {
  for (size_t k = 0; k < sizeof(pairs)/sizeof(pairs[0]); k++) {
    int sda = pairs[k][0];
    int scl = pairs[k][1];

    Serial.print("\nTrying SDA=");
    Serial.print(sda);
    Serial.print("  SCL=");
    Serial.println(scl);

    Wire.begin(sda, scl);
    delay(10);

    int foundCount = 0;
    for (byte address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      byte error = Wire.endTransmission();
      if (error == 0) {
        Serial.print("  Device found at 0x");
        if (address < 16) Serial.print("0");
        Serial.print(address, HEX);
        Serial.println();
        foundCount++;
      }
    }

    if (foundCount == 0) {
      Serial.println("  No devices found on this pair.");
    }

    delay(300);
    Wire.end();
    delay(100);
  }

  Serial.println("\nScan complete. Waiting 5s before re-run...");
  delay(5000);
}
