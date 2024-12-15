#include "ADS1115.h"

// Create an instance of the ADS1115 class
ADS1115 ads;

// Known reference voltage (from TL431)
const float TL431_VOLTAGE = 2.5;

// Shunt resistor and NCS199 gain
const float SHUNT_RESISTOR = 0.001; // 1mΩ
const float NCS_GAIN = 50.0;        // Gain of NCS199-A1

// Calibration coefficients
const float CALIBRATION_SLOPE = 0.8703; // Slope (m)
const float CALIBRATION_OFFSET = 0.1479; // Offset (b)

void setup() {
    Serial.begin(9600);
    ads.begin();

    // Configure the ADS1115 with:
    // - ±6.144V range to handle 0–5V inputs
    // - 128 samples per second
    // - Single-shot mode for explicit control of readings
    ads.configure(ADS1115_PGA_6_144V, ADS1115_DATARATE_128SPS, ADS1115_MODE_SINGLESHOT);
    Serial.println("Current Measurement with Calibration");
    Serial.println("----------------------------------------");
}

void loop() {
    printDivider();
    // Read Channel 1: Current measurement with ±2.5V reference
    int16_t raw1 = ads.readRaw(0);
    float voltage1 = ads.rawToVoltage(raw1);
    float arduinoCurrent = calculateCurrent(voltage1, TL431_VOLTAGE, NCS_GAIN, SHUNT_RESISTOR);
    float calibratedCurrent = calibrateCurrent(arduinoCurrent);

    Serial.println("Channel 1:");
    Serial.print("  Raw ADC: ");
    Serial.print(raw1);
    Serial.print(", Voltage: ");
    Serial.print(voltage1, 6);
    Serial.print(" V, Arduino Current: ");
    Serial.print(arduinoCurrent, 6);
    Serial.print(" A, Calibrated Current: ");
    Serial.print(calibratedCurrent, 6);
    Serial.println(" A");

    // Read Channel 2: Current measurement with 0V reference
    int16_t raw2 = ads.readRaw(1);
    float voltage2 = ads.rawToVoltage(raw2);
    float current2 = calculateCurrent(voltage2, 0.0, NCS_GAIN, 0.05); // Gain = 50, Shunt = 50mΩ

    Serial.println("Channel 2:");
    Serial.print("  Raw ADC: ");
    Serial.print(raw2);
    Serial.print(", Voltage: ");
    Serial.print(voltage2, 6);
    Serial.print(" V, Current: ");
    Serial.print(current2, 6);
    Serial.println(" A");

    // Read Channel 3: Voltage measurement through divider (22/144 ratio)
    int16_t raw3 = ads.readRaw(2);
    float voltage3 = ads.rawToVoltage(raw3);
    float actualVoltage3 = calculateVoltage(voltage3, 22.0 / 144.0); // Divider ratio = 22/144

    Serial.println("Channel 3:");
    Serial.print("  Raw ADC: ");
    Serial.print(raw3);
    Serial.print(", Voltage: ");
    Serial.print(voltage3, 6);
    Serial.print(" V, Actual Voltage: ");
    Serial.print(actualVoltage3, 6);
    Serial.println(" V");

    // Read Channel 4: Voltage measurement through divider (22/144 ratio)
    int16_t raw4 = ads.readRaw(3);
    float voltage4 = ads.rawToVoltage(raw4);
    float actualVoltage4 = calculateVoltage(voltage4, 22.0 / 144.0); // Divider ratio = 22/144

    Serial.println("Channel 4:");
    Serial.print("  Raw ADC: ");
    Serial.print(raw4);
    Serial.print(", Voltage: ");
    Serial.print(voltage4, 6);
    Serial.print(" V, Actual Voltage: ");
    Serial.print(actualVoltage4, 6);
    Serial.println(" V");

    printDivider();
    delay(1000); // Delay between readings
}

// Function to calculate current
float calculateCurrent(float measuredVoltage, float referenceVoltage, float gain, float shuntResistor) {
    return (measuredVoltage - referenceVoltage) / (gain * shuntResistor);
}

// Function to calibrate current
float calibrateCurrent(float arduinoCurrent) {
    return (CALIBRATION_SLOPE * arduinoCurrent) + CALIBRATION_OFFSET;
}

// Function to calculate voltage
float calculateVoltage(float measuredVoltage, float dividerGain) {
    return measuredVoltage / dividerGain;
}

// Helper function to print a divider line
void printDivider() {
    Serial.println("----------------------------------------");
}
