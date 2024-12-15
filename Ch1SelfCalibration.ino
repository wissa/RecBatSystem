#include "ADS1115.h"
#include <EEPROM.h>

// Create an instance of the ADS1115 class
ADS1115 ads;

// Known reference voltage (from TL431)
const float TL431_VOLTAGE = 2.5;

// Shunt resistor and NCS199 gain
const float SHUNT_RESISTOR = 0.001; // 1mΩ
const float NCS_GAIN = 50.0;        // Gain of NCS199-A1

// EEPROM Addresses
const int EEPROM_SLOPE_ADDR = 0;    // Address to store slope
const int EEPROM_OFFSET_ADDR = 4;  // Address to store offset

// Calibration coefficients
float calibrationSlope = 1.0;  // Default slope
float calibrationOffset = 0.0; // Default offset

void setup() {
    Serial.begin(9600);
    ads.begin();

    // Configure the ADS1115 with:
    // - ±6.144V range to handle 0–5V inputs
    // - 128 samples per second
    // - Single-shot mode for explicit control of readings
    ads.configure(ADS1115_PGA_6_144V, ADS1115_DATARATE_128SPS, ADS1115_MODE_SINGLESHOT);

    // Load calibration parameters from EEPROM
    EEPROM.get(EEPROM_SLOPE_ADDR, calibrationSlope);
    EEPROM.get(EEPROM_OFFSET_ADDR, calibrationOffset);

    // Handle uninitialized EEPROM values
    if (isnan(calibrationSlope) || isnan(calibrationOffset)) {
        calibrationSlope = 1.0;
        calibrationOffset = 0.0;
    }

    Serial.println("Current Measurement with Self-Calibration");
    Serial.println("----------------------------------------");
    Serial.print("Loaded Calibration Slope: ");
    Serial.println(calibrationSlope, 6);
    Serial.print("Loaded Calibration Offset: ");
    Serial.println(calibrationOffset, 6);
}

void loop() {
    printDivider();

    // Measure current for Channel 1
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

    printDivider();

    // Trigger self-calibration when 'c' is received in the serial monitor
    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'c') { // Trigger self-calibration
            selfCalibrate();
        }
    }

    delay(1000); // Delay between readings
}

// Function to calculate current
float calculateCurrent(float measuredVoltage, float referenceVoltage, float gain, float shuntResistor) {
    return (measuredVoltage - referenceVoltage) / (gain * shuntResistor);
}

// Function to calibrate current
float calibrateCurrent(float arduinoCurrent) {
    return (calibrationSlope * arduinoCurrent) + calibrationOffset;
}

// Self-calibration function
void selfCalibrate() {
    Serial.println("Starting Self-Calibration...");
    Serial.println("The system will collect 20 points. Provide known currents (in A) when prompted.");
    Serial.println("----------------------------------------");

    float arduinoCurrents[20];
    float knownCurrents[20];
    int dataIndex = 0;

    while (dataIndex < 20) {
        // Read current from Channel 1
        int16_t raw = ads.readRaw(0);
        float voltage = ads.rawToVoltage(raw);
        float arduinoCurrent = calculateCurrent(voltage, TL431_VOLTAGE, NCS_GAIN, SHUNT_RESISTOR);

        // Display the Arduino-calculated current
        Serial.print("Point ");
        Serial.print(dataIndex + 1);
        Serial.print(": Arduino Current = ");
        Serial.print(arduinoCurrent, 6);
        Serial.println(" A");

        // Ask the user to enter the known current
        Serial.print("Enter the known current (in A): ");
        while (!Serial.available()); // Wait for user input
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove leading/trailing whitespace

        // Convert input to a float
        float knownCurrent = input.toFloat();
        if (!isnan(knownCurrent) && knownCurrent >= 0) {
            arduinoCurrents[dataIndex] = arduinoCurrent;
            knownCurrents[dataIndex] = knownCurrent;

            dataIndex++;
        } else {
            Serial.println("Invalid input. Please enter a valid positive current value.");
        }
    }

    // Perform calibration using linear regression
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < dataIndex; i++) {
        sumX += arduinoCurrents[i];
        sumY += knownCurrents[i];
        sumXY += arduinoCurrents[i] * knownCurrents[i];
        sumX2 += arduinoCurrents[i] * arduinoCurrents[i];
    }

    calibrationSlope = (dataIndex * sumXY - sumX * sumY) / (dataIndex * sumX2 - sumX * sumX);
    calibrationOffset = (sumY - calibrationSlope * sumX) / dataIndex;

    // Print the new calibration coefficients
    Serial.println("Calibration Complete!");
    Serial.print("New Slope: ");
    Serial.println(calibrationSlope, 6);
    Serial.print("New Offset: ");
    Serial.println(calibrationOffset, 6);

    // Save the calibration coefficients to EEPROM
    EEPROM.put(EEPROM_SLOPE_ADDR, calibrationSlope);
    EEPROM.put(EEPROM_OFFSET_ADDR, calibrationOffset);
    Serial.println("Calibration parameters saved to EEPROM.");
}

// Helper function to print a divider line
void printDivider() {
    Serial.println("----------------------------------------");
}
