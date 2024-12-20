#include "ADS1115.h"
#include <EEPROM.h>
#include <Wire.h>

//OUTPUTs
const int SyncPin = 2; 
const int Pch = 5;   
const int Pdch = 6; 
const int MBal = 3; 
const int LED1 = 7; 
const int LED2 = 4; 

const int CSW1 = 8;
const int CSW2 = 9;
const int CSW3 = 10;
const int CSW4 = 11;
const int CSW5 = 12;
const int CSW6 = 13;
const int CSW7 = 20;
const int CSW8 = 21;

//INPUTs
const int add0 = 25; 
const int add1 = 26; 
const int add2 = 14; 
const int add3 = 15; 

//const int Vs = 14;     //A0
//const int VT = 15;      //A1
const int Tb = 16;      //A2
const int REF25 = 17;      //A3 
//////////////

ADS1115 ads;


// Known reference voltage (from TL431)
const float TL431_VOLTAGE = 2.5;

// Shunt resistor and NCS199 gain
const float SHUNT_RESISTOR = 0.001; // 1mΩ
const float NCS_GAIN = 50.0;        // Gain of NCS199-A1

// Divider ratios for channels 3 and 4
const float DIVIDER_RATIO = 22.0 / 144.0;

// EEPROM Addresses for calibration parameters
const int EEPROM_BASE_ADDR = 0; // Base address
const int SLOPE_OFFSET_SIZE = 8; // Size for each channel's slope and offset
// EEPROM Addresses for Battery Settings
const int FULL_VOLTAGE_ADDR = 36;  // Fully charged voltage storage
const int EMPTY_VOLTAGE_ADDR = 40; // Fully discharged voltage storage
const int BAT_CAPACITY_ADDR = 44;


// Calibration coefficients
float calibrationSlope[4] = {1.0, 1.0, 1.0, 1.0};
float calibrationOffset[4] = {0.0, 0.0, 0.0, 0.0};
////////////////////

float BatCurrent, BatVoltage, BalCurrent, TerVoltage;

////////////////////
// Battery parameters specific to LFP
float fullVoltage = 3.4;  // Fully charged voltage for LFP
float emptyVoltage = 2.5; // Fully discharged voltage for LFP
float batteryCapacity = 14000.0; // Battery capacity in mAh



// Kalman Filter variables
float Q = 0.001; // Process noise covariance
float R = 0.1;   // Measurement noise covariance
float P = 1.0;   // Estimate error covariance
float K = 0.0;   // Kalman gain
float soc_estimate = 100.0; // Initial SoC estimate

unsigned long previousTime = 0; // Time tracking for Coulomb Counting

void onI2CRequest();


void setup() {
  // put your setup code here, to run once:

pinMode(SyncPin, INPUT_PULLUP);
pinMode(Pch, OUTPUT);
pinMode(Pdch, OUTPUT);
pinMode(MBal, OUTPUT);
pinMode(LED1, OUTPUT);
pinMode(LED2, OUTPUT);

pinMode(CSW1, OUTPUT);
pinMode(CSW2, OUTPUT);
pinMode(CSW3, OUTPUT);
pinMode(CSW4, OUTPUT);
pinMode(CSW5, OUTPUT);
pinMode(CSW6, OUTPUT);
pinMode(CSW7, OUTPUT);
pinMode(CSW8, OUTPUT);

pinMode(add0, INPUT);
pinMode(add1, INPUT);
pinMode(add2, INPUT);
pinMode(add3, INPUT);

///////////////////////////////
  Serial.begin(9600);       //  <<<<<<<<<  fast!

    ads.begin();

    // Configure the ADS1115
    ads.configure(ADS1115_PGA_6_144V, ADS1115_DATARATE_128SPS, ADS1115_MODE_SINGLESHOT);

    // Load calibration coefficients from EEPROM
    for (int i = 0; i < 4; i++) {
        EEPROM.get(EEPROM_BASE_ADDR + i * SLOPE_OFFSET_SIZE, calibrationSlope[i]);
        EEPROM.get(EEPROM_BASE_ADDR + i * SLOPE_OFFSET_SIZE + 4, calibrationOffset[i]);

        // Handle uninitialized EEPROM values
        if (isnan(calibrationSlope[i]) || isnan(calibrationOffset[i])) {
            calibrationSlope[i] = 1.0;
            calibrationOffset[i] = 0.0;
        }
    }
        // Getting Battery setting
        EEPROM.get(FULL_VOLTAGE_ADDR, fullVoltage);
        EEPROM.get(EMPTY_VOLTAGE_ADDR, emptyVoltage);
        EEPROM.get(BAT_CAPACITY_ADDR, batteryCapacity);

        // Handle uninitialized EEPROM values
        if (isnan(fullVoltage) || isnan(emptyVoltage) || isnan(batteryCapacity)) {
            fullVoltage = 3.4;
            emptyVoltage = 2.5;
            batteryCapacity = 14000;
        }


 previousTime = millis(); // Initialize time tracking for SoC estimation
Serial.println("ADS1115 Measurement with Per-Channel Self-Calibration");
    Serial.println("----------------------------------------");
    for (int i = 0; i < 4; i++) {
        Serial.print("Channel ");
        Serial.print(i + 1);
        Serial.print(": Slope = ");
        Serial.print(calibrationSlope[i], 6);
        Serial.print(", Offset = ");
        Serial.println(calibrationOffset[i], 6);
    }


Wire.begin(0x11); // Start I2C port 1 as slave with address 0x11
Wire.setClock(400000);
Wire.onRequest(onI2CRequest); // When master requests data, call onI2CRequest()

}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(Pch, HIGH);
digitalWrite(Pdch, HIGH);
digitalWrite(LED2, HIGH);
analogWrite(MBal, 0);

// Process each channel
    for (int i = 0; i < 4; i++) {
        processChannel(i);
    }

    printDivider();

    // Update SoC using Kalman Filter
    updateSoC(BatVoltage, BatCurrent);

    // Print the SoC status
    printSoCStatus(BatVoltage, BatCurrent, soc_estimate);

//////////////////////////////////////////////////

// Check for calibration commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // Read the full command
        command.trim(); // Remove any extra whitespace

        if (command == "c1") {
            selfCalibrate(0); // Calibrate Channel 1
        } else if (command == "c2") {
            selfCalibrate(1); // Calibrate Channel 2
        } else if (command == "c3") {
            selfCalibrate(2); // Calibrate Channel 3
        } else if (command == "c4") {
            selfCalibrate(3); // Calibrate Channel 4
        }
        if (command == "b") {
            enterBatterySettingMode(); // Append Battery Setting Mode
        }
    }

  digitalWrite(LED2, HIGH);
  delay(500);
  digitalWrite(LED2, LOW);
  delay(500);
}


// Process a single channel
void processChannel(int channel) {
    int16_t raw = ads.readRaw(channel);
    float voltage = ads.rawToVoltage(raw);

    Serial.print("Channel ");
    Serial.print(channel + 1);
    Serial.print(": ");
    Serial.print("Raw ADC: ");
    Serial.print(raw);
    Serial.print(", Voltage: ");
    Serial.print(voltage, 6);

    if (channel == 0) { // Channel 1: Current measurement with ±2.5V reference
        float arduinoCurrent = calculateCurrent(voltage, TL431_VOLTAGE, NCS_GAIN, SHUNT_RESISTOR);
        float calibratedCurrent = calibrateCurrent(channel, arduinoCurrent);
        if (calibratedCurrent<0.001) {calibratedCurrent=0;}
        BatCurrent = calibratedCurrent;
        Serial.print(", Arduino Current: ");
        Serial.print(arduinoCurrent, 6);
        Serial.print(" A, Calibrated Current: ");
        Serial.print(calibratedCurrent, 6);
        Serial.println(" A");
    } else if (channel == 1) { // Channel 2: Current measurement with 0V reference
        float current = calculateCurrent(voltage, 0.0, NCS_GAIN, 0.05); // Shunt = 50mΩ
        float calibratedCurrent = calibrateCurrent(channel, current);
        if (calibratedCurrent<0.001) {calibratedCurrent=0;}
        BalCurrent = calibratedCurrent;
        Serial.print(", Current: ");
        Serial.print(current, 6);
        Serial.print(" A, Calibrated Current: ");
        Serial.print(calibratedCurrent, 6);
        Serial.println(" A");
    } else { // Channels 3 and 4: Voltage measurement
        float actualVoltage = calculateVoltage(voltage, DIVIDER_RATIO);
        float calibratedVoltage = calibrateCurrent(channel, actualVoltage); // Using same calibration logic
        if (channel == 2) {BatVoltage = calibratedVoltage;}
        if (channel == 3) {TerVoltage = calibratedVoltage;}
        Serial.print(", Actual Voltage: ");
        Serial.print(actualVoltage, 6);
        Serial.print(" V, Calibrated Voltage: ");
        Serial.print(calibratedVoltage, 6);
        Serial.println(" V");
    }
}

// Function to calculate current
float calculateCurrent(float measuredVoltage, float referenceVoltage, float gain, float shuntResistor) {
    return (measuredVoltage - referenceVoltage) / (gain * shuntResistor);
}

// Function to calibrate a value (current or voltage)
float calibrateCurrent(int channel, float value) {
    return (calibrationSlope[channel] * value) + calibrationOffset[channel];
}

// Function to calculate voltage
float calculateVoltage(float measuredVoltage, float dividerGain) {
    return measuredVoltage / dividerGain;
}

// Self-calibration function for a specific channel
void selfCalibrate(int channel) {
    Serial.print("Starting Self-Calibration for Channel ");
    Serial.print(channel + 1);
    Serial.println("...");
    Serial.println("Provide known values for calibration. 20 points required.");

    float arduinoValues[20];
    float knownValues[20];
    int dataIndex = 0;

    while (dataIndex < 20) {
        // Measure the Arduino-calculated value
        int16_t raw = ads.readRaw(channel);
        float voltage = ads.rawToVoltage(raw);
        float arduinoValue;

        if (channel == 0) {
            arduinoValue = calculateCurrent(voltage, TL431_VOLTAGE, NCS_GAIN, SHUNT_RESISTOR);
        } else if (channel == 1) {
            arduinoValue = calculateCurrent(voltage, 0.0, NCS_GAIN, 0.05);
        } else {
            arduinoValue = calculateVoltage(voltage, DIVIDER_RATIO);
        }

        // Display the Arduino-calculated value
        Serial.print("Point ");
        Serial.print(dataIndex + 1);
        Serial.print(": Arduino Value = ");
        Serial.print(arduinoValue, 6);
        Serial.println();

        // Ask the user for the known value
        Serial.print("Enter the known value: ");
        while (!Serial.available()); // Wait for user input
        String input = Serial.readStringUntil('\n');
        input.trim();

        // Validate the input
        float knownValue = input.toFloat();
        if (!isnan(knownValue) && knownValue >= 0) {
            arduinoValues[dataIndex] = arduinoValue;
            knownValues[dataIndex] = knownValue;
            dataIndex++;
        } else {
            Serial.println("Invalid input. Try again.");
        }
    }

    // Perform linear regression for calibration
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < 20; i++) {
        sumX += arduinoValues[i];
        sumY += knownValues[i];
        sumXY += arduinoValues[i] * knownValues[i];
        sumX2 += arduinoValues[i] * arduinoValues[i];
    }

    calibrationSlope[channel] = (20 * sumXY - sumX * sumY) / (20 * sumX2 - sumX * sumX);
    calibrationOffset[channel] = (sumY - calibrationSlope[channel] * sumX) / 20;

    // Save to EEPROM
    EEPROM.put(EEPROM_BASE_ADDR + channel * SLOPE_OFFSET_SIZE, calibrationSlope[channel]);
    EEPROM.put(EEPROM_BASE_ADDR + channel * SLOPE_OFFSET_SIZE + 4, calibrationOffset[channel]);

    Serial.print("Calibration for Channel ");
    Serial.print(channel + 1);
    Serial.println(" Complete!");
    Serial.print("New Slope: ");
    Serial.println(calibrationSlope[channel], 6);
    Serial.print("New Offset: ");
    Serial.println(calibrationOffset[channel], 6);
}

// Helper function to print a divider line
void printDivider() {
    Serial.println("----------------------------------------");
}

//////////////////////////////// SOC Estimation ////////////////////////////////
void updateSoC(float measuredVoltage, float current) {
    unsigned long currentTime = millis();
    float elapsedHours = (currentTime - previousTime) / 3600000.0; // Convert ms to hours
    previousTime = currentTime;

    // Update SoC using Coulomb Counting
    float deltaSOC = (current * elapsedHours) / batteryCapacity * 100.0;
    soc_estimate -= deltaSOC;

    // Apply Kalman Filter for voltage-based correction
    // Predict phase
    P = P + Q;

    // Update phase
    K = P / (P + R);
    float soc_from_voltage = voltageToSOC(measuredVoltage);
    soc_estimate = soc_estimate + K * (soc_from_voltage - soc_estimate);
    P = (1 - K) * P;

    // Constrain SoC between 0% and 100%
    soc_estimate = constrain(soc_estimate, 0.0, 100.0);
}

////////////////////

float voltageToSOC(float voltage) {
    if (voltage >= fullVoltage) return 100.0;
    if (voltage <= emptyVoltage) return 0.0;
    return (voltage - emptyVoltage) / (fullVoltage - emptyVoltage) * 100.0;
}

////////////////////

void printSoCStatus(float voltage, float current, float soc) {
    Serial.println("----------------------------------------");
    Serial.print("Battery Voltage: ");
    Serial.print(voltage, 6);
    Serial.print(" V, Battery Current: ");
    Serial.print(current, 6);
    Serial.print(" A, SoC: ");
    Serial.print(soc, 1);
    Serial.println(" %");
    Serial.println("----------------------------------------");
}

////////////////////

// ***************************
// Battery Setting Mode Section
// ***************************
void enterBatterySettingMode() {
    Serial.println("Entering Battery Setting Mode...");

    // Battery Type Selection
    Serial.println("Select Battery Type: ");
    Serial.println("1: LFP (3.4V full, 2.5V empty)");
    Serial.println("2: Li-ion (4.2V full, 3.0V empty)");
    while (!Serial.available());
    int batteryType = Serial.parseInt();
    Serial.readString();

    if (batteryType == 1) {
        fullVoltage = 3.4;
        emptyVoltage = 2.5;
        Serial.println("LFP Battery Selected.");
    } else if (batteryType == 2) {
        fullVoltage = 4.2;
        emptyVoltage = 3.0;
        Serial.println("Li-ion Battery Selected.");
    } else {
        Serial.println("Invalid input. Defaulting to LFP.");
        fullVoltage = 3.4;
        emptyVoltage = 2.5;
    }

    // Custom Voltage Input
    Serial.println("Enter Fully Charged Voltage:");
    while (!Serial.available());
    fullVoltage = Serial.parseFloat();
    Serial.readString();

    Serial.println("Enter Fully Discharged Voltage:");
    while (!Serial.available());
    emptyVoltage = Serial.parseFloat();
    Serial.readString();

    Serial.println("Enter Battery Capacity:");
    while (!Serial.available());
    batteryCapacity = Serial.parseFloat();
    Serial.readString();

    // Save to EEPROM
    EEPROM.put(FULL_VOLTAGE_ADDR, fullVoltage);
    EEPROM.put(EMPTY_VOLTAGE_ADDR, emptyVoltage);
    EEPROM.put(BAT_CAPACITY_ADDR, batteryCapacity);

    Serial.println("Settings Saved:");
    Serial.print("Full Voltage: ");
    Serial.println(fullVoltage, 2);
    Serial.print("Empty Voltage: ");
    Serial.println(emptyVoltage, 2);
    Serial.println("Exiting Battery Setting Mode...");
}

void onI2CRequest() {
  // Convert floats to bytes and send them
  // We'll send [BatVoltage][BatCurrent][SoC] each 4 bytes = total 12 bytes

  Wire.write((byte*)&BatVoltage, sizeof(BatVoltage));
  Wire.write((byte*)&BatCurrent, sizeof(BatCurrent));
  Wire.write((byte*)&soc_estimate, sizeof(soc_estimate));
}


