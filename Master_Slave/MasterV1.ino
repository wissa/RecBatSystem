#include <Wire.h>

#define SLAVE_ADDR 0x11

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Master mode
  Wire.setClock(400000);
  delay(1000);
  Serial.println("Master ready.");
}

void loop() {
  // Request 12 bytes from the slave: 3 floats * 4 bytes each
  Wire.requestFrom(SLAVE_ADDR, 20);

  byte buffer[20];
  int idx = 0;
  while (Wire.available() && idx < 20) {
    buffer[idx++] = Wire.read();
  }

  if (idx == 20) {
    // Reconstruct floats
    float receivedBatVoltage, receivedBatCurrent, receivedSoC, receivedTemp, receivedVcc ;
    memcpy(&receivedBatVoltage, &buffer[0], 4);
    memcpy(&receivedBatCurrent, &buffer[4], 4);
    memcpy(&receivedSoC, &buffer[8], 4);
    memcpy(&receivedTemp, &buffer[12], 4);
    memcpy(&receivedVcc, &buffer[16], 4);

    Serial.print("Voltage: "); Serial.print(receivedBatVoltage, 2);
    Serial.print(" V, Current: "); Serial.print(receivedBatCurrent, 2);
    Serial.print(" A, SoC: "); Serial.print(receivedSoC, 1);
    Serial.println(" %");
    Serial.print("Vcc: "); Serial.print(receivedVcc, 2);
    Serial.println(" V");
    Serial.print("Temp: "); Serial.print(receivedTemp, 2);
    Serial.println(" C");   
  } else {
    Serial.println("Did not receive full data from slave!");
  }

  delay(500);
}
