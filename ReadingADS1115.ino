#include "ADS1115.h"

// Create an instance of the ADS1115 class
ADS1115 ads;

void setup() {
    Serial.begin(9600);
    ads.begin(); // Initialize the ADS1115
}

void loop() {
    // Read and convert data for each channel
    float current1 = ads.rawToCurrent(ads.readRaw(0), 2.5, 50, 0.001); // Channel 1
    float current2 = ads.rawToCurrent(ads.readRaw(1), 0, 50, 0.05);    // Channel 2
    float voltage3 = ads.rawToVoltage(ads.readRaw(2)) / (22.0 / 144.0); // Channel 3
    float voltage4 = ads.rawToVoltage(ads.readRaw(3)) / (22.0 / 144.0); // Channel 4

    // Print the results
    Serial.print("Current CH1: ");
    Serial.print(current1);
    Serial.println(" A");

    Serial.print("Current CH2: ");
    Serial.print(current2);
    Serial.println(" A");

    Serial.print("Voltage CH3: ");
    Serial.print(voltage3);
    Serial.println(" V");

    Serial.print("Voltage CH4: ");
    Serial.print(voltage4);
    Serial.println(" V");

    delay(1000); // Wait before the next reading
}
