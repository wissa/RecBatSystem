#include "ADS1115.h"
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>


ADS1115 ads;

struct BatteryConfig {
  uint8_t batteryType;   // 1=LFP, 2=Li-ion
  float fullVoltage;     
  float emptyVoltage;    
  float batteryCapacity;        
};

struct ProtectionConfig {
  float I_chmax;       // Max. Charging Current
  float I_dchmax;       // Max. DisCharging Current
  float OCVP; // Over Charge voltage protection
  float ODVP; // Over DisCharge voltage protection     
  float I_Bal; // Balance current setting   
};

struct Switch_Config {
  uint8_t SW_Config;   // switches settings
};

BatteryConfig BatConfig;
ProtectionConfig ProtConfig;
Switch_Config MySW_Config;

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

//////////////
// Known reference voltage (from TL431)
const float TL431_VOLTAGE = 2.5;

// Shunt resistor and NCS199 gain
const float SHUNT_RESISTOR = 0.001; // 1mΩ
const float NCS_GAIN = 50.0;        // Gain of NCS199-A1

// Divider ratios for channels 3 and 4
const float DIVIDER_RATIO = 22.0 / 144.0;

uint8_t  FaultReg=3;
// EEPROM Addresses for calibration parameters
const int EEPROM_BASE_ADDR = 0; // Base address
const int SLOPE_OFFSET_SIZE = 8; // Size for each channel's slope and offset
// EEPROM Addresses for Battery Settings
const int FULL_VOLTAGE_ADDR = 36;  // Fully charged voltage storage
const int EMPTY_VOLTAGE_ADDR = 40; // Fully discharged voltage storage
const int BAT_CAPACITY_ADDR = 44;
const int BAT_TYPE_ADDR = 48;
// EEPROM Addresses for Protection Settings
const int I_chmax_ADDR = 56; 
const int I_dchmax_ADDR = 60; 
const int OCVP_ADDR = 64;
const int ODVP_ADDR = 68;
const int I_Bal_ADDR = 72;
// EEPROM SW setting
const int SW_ADDR = 80;

float DefaultfullVoltage = 3.4;  // Fully charged voltage for LFP
float DefaultemptyVoltage = 2.5; // Fully discharged voltage for LFP
float DefaultbatteryCapacity = 14000.0; // Battery capacity in mAh

// Calibration coefficients
float calibrationSlope[4] = {1.0, 1.0, 1.0, 1.0};
float calibrationOffset[4] = {0.0, 0.0, 0.0, 0.0};
////////////////////

float BatCurrent, BatVoltage, BalCurrent, TerVoltage;

////////////////////
// Kalman Filter variables
float Q = 0.001; // Process noise covariance
float R = 0.1;   // Measurement noise covariance
float P = 1.0;   // Estimate error covariance
float K = 0.0;   // Kalman gain
float soc_estimate = 100.0; // Initial SoC estimate

unsigned long previousTime = 0; // Time tracking for Coulomb Counting

#define THERMISTOR_PIN A2
#define REF_PIN A3

// Thermistor parameters
#define R_FIXED 10000.0    // Fixed resistor 10kΩ
#define R0      47000.0    // NTC resistance at 25°C
#define T0      298.15     // Kelvin (25°C)
#define BETA    4108.0     // Beta coefficient
#define ADC_MAX 1023.0

// To integrate into the I2C response, we add new global variables:
float currentTemperatureC = 0.0;
float currentVcc = 0.0;

// This variable stores the last command received from the Master
char lastCommand = '\0';

void onI2CRequest();
void onI2CReceive();

//////////////////////////////////////////////////////////////////
void onI2CRequest() {
  // Convert floats to bytes and send them
  // We'll send [BatVoltage][BatCurrent][SoC] each 4 bytes = total 12 bytes
if (lastCommand == 'A') {
  sendFloat(BatVoltage);
  sendFloat(BatCurrent);
  sendFloat(soc_estimate);
  sendFloat(currentTemperatureC);
  sendFloat(currentVcc);
  sendFloat(BalCurrent);
  sendFloat(TerVoltage);
  sendFloat(FaultReg);
  Wire.write(FaultReg);
  lastCommand = 0;
}

if (lastCommand == 'B') {

  EEPROM.get(FULL_VOLTAGE_ADDR,BatConfig.fullVoltage);
  EEPROM.get(EMPTY_VOLTAGE_ADDR,BatConfig.emptyVoltage);
  EEPROM.get(BAT_CAPACITY_ADDR,BatConfig.batteryCapacity);
  EEPROM.get(BAT_TYPE_ADDR,BatConfig.batteryType);
  Wire.write(BatConfig.batteryType);
  sendFloat(BatConfig.fullVoltage);
  sendFloat(BatConfig.emptyVoltage);
  sendFloat(BatConfig.batteryCapacity);
  lastCommand = 0;
}


////// Send Protection Settings //////////
if (lastCommand == 'P') {

  EEPROM.get(I_chmax_ADDR,ProtConfig.I_chmax);
  EEPROM.get(I_dchmax_ADDR,ProtConfig.I_dchmax);
  EEPROM.get(OCVP_ADDR,ProtConfig.OCVP);
  EEPROM.get(ODVP_ADDR,ProtConfig.ODVP);
  EEPROM.get(I_Bal_ADDR, ProtConfig.I_Bal);
  sendFloat(ProtConfig.I_chmax);
  sendFloat(ProtConfig.I_dchmax);
  sendFloat(ProtConfig.OCVP);
  sendFloat(ProtConfig.ODVP);
  sendFloat(ProtConfig.I_Bal);
  lastCommand = 0;
}
}

// Called automatically when Master sends data to Slave
void onI2CReceive(int numBytes) {
  // Read the command (assuming Master sends exactly one byte for the command)
  if (numBytes > 0) {
    lastCommand = Wire.read();  // e.g. 'V', 'I', 'S', 'A', 'L', etc.
    //Serial.println(lastCommand);

//// Setting Battery Setting //////////////
if (lastCommand == 'C') {

   if (numBytes < 1+12) {
    // Serial.print (numBytes);
    //  Serial.println("Error: Not enough data for battery config.");
      return;
    }
   // Serial.println("New battery config from Master");
    BatConfig.batteryType = Wire.read();
    numBytes--;
    BatConfig.fullVoltage  = readFloat();
    BatConfig.emptyVoltage = readFloat();
    BatConfig.batteryCapacity = readFloat();

  EEPROM.put(FULL_VOLTAGE_ADDR,BatConfig.fullVoltage);
  EEPROM.put(EMPTY_VOLTAGE_ADDR,BatConfig.emptyVoltage);
  EEPROM.put(BAT_CAPACITY_ADDR,BatConfig.batteryCapacity);
  EEPROM.put(BAT_TYPE_ADDR,BatConfig.batteryType);
  //Serial.println("New battery config received and saved to EEPROM");
  lastCommand = 0;
}

//// Setting Protection Setting //////////////
if (lastCommand == 'Q') {

   if (numBytes < 1+20) {
    // Serial.print (numBytes);
     // Serial.println("Error: Not enough data for Protection config.");
      return;
    }
   // Serial.println("New Protection settings from Master");
    ProtConfig.I_chmax = readFloat();
    ProtConfig.I_dchmax  = readFloat();
    ProtConfig.OCVP = readFloat();
    ProtConfig.ODVP = readFloat();
    ProtConfig.I_Bal = readFloat();

    EEPROM.put(I_chmax_ADDR, ProtConfig.I_chmax);
    EEPROM.put(I_dchmax_ADDR, ProtConfig.I_dchmax);
    EEPROM.put(OCVP_ADDR, ProtConfig.OCVP);
    EEPROM.put(ODVP_ADDR, ProtConfig.ODVP);
    EEPROM.put(I_Bal_ADDR, ProtConfig.I_Bal);
  //Serial.println("New Protection config received and saved to EEPROM");
  lastCommand = 0;
}
//// Setting Swtiches setting //////////////
if (lastCommand == 'S') {
   if (numBytes < 1+1) {
    // Serial.print (numBytes);
    //  Serial.println("Error: Not enough data for Swtichtes config.");
      return;
    }
    //Serial.println("New Swtiches settings from Master");
    MySW_Config.SW_Config = Wire.read();
    EEPROM.put(SW_ADDR, MySW_Config.SW_Config);
  //Serial.print("New Swtichtes config received and saved to EEPROM:");
  //Serial.print(MySW_Config.SW_Config, BIN);
  //Serial.print(MySW_Config.SW_Config.b[0]);
  lastCommand = 0;
    }
  }
}

/////////////////////////////////////////////////////////////////////////
void UpdateSW() {
  // Copy the value of mySwConfig.SW_Config to PORTB
  PORTB = MySW_Config.SW_Config;
    Serial.println("Interrupt triggered: PORTB updated.");
}
//////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
pinMode(SyncPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(SyncPin), UpdateSW, FALLING);
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

//DDRB = 0xFF;  
///////////////////////////////
  Serial.begin(9600);    
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
        EEPROM.get(FULL_VOLTAGE_ADDR, BatConfig.fullVoltage);
        EEPROM.get(EMPTY_VOLTAGE_ADDR, BatConfig.emptyVoltage);
        EEPROM.get(BAT_CAPACITY_ADDR, BatConfig.batteryCapacity);
        EEPROM.get(BAT_TYPE_ADDR, BatConfig.batteryType);

        // Handle uninitialized EEPROM values
        if (isnan(BatConfig.fullVoltage) || isnan(BatConfig.emptyVoltage) || isnan(BatConfig.batteryCapacity)) {
            BatConfig.fullVoltage = DefaultfullVoltage;
            BatConfig.emptyVoltage = DefaultemptyVoltage;
            BatConfig.batteryCapacity = DefaultbatteryCapacity;
            BatConfig.batteryType = 1;
            EEPROM.put(FULL_VOLTAGE_ADDR, BatConfig.fullVoltage);
            EEPROM.put(EMPTY_VOLTAGE_ADDR, BatConfig.emptyVoltage);
            EEPROM.put(BAT_CAPACITY_ADDR, BatConfig.batteryCapacity);
            EEPROM.put(BAT_TYPE_ADDR, BatConfig.batteryType);

        }

        //Getting Protection Setting
          EEPROM.get(I_chmax_ADDR,ProtConfig.I_chmax);
          EEPROM.get(I_dchmax_ADDR,ProtConfig.I_dchmax);
          EEPROM.get(OCVP_ADDR,ProtConfig.OCVP);
          EEPROM.get(ODVP_ADDR,ProtConfig.ODVP);
          EEPROM.get(I_Bal_ADDR,ProtConfig.I_Bal);

                  // Handle uninitialized EEPROM values
        if (isnan(ProtConfig.I_chmax) || isnan(ProtConfig.I_dchmax) || isnan(ProtConfig.OCVP) || isnan(ProtConfig.ODVP || ProtConfig.I_Bal)) {
            ProtConfig.I_chmax = 1;
            ProtConfig.I_dchmax = 2;
            ProtConfig.OCVP = 3.55;
            ProtConfig.ODVP = 2.6;
            ProtConfig.I_Bal = 0.3;
            EEPROM.put(I_chmax_ADDR, ProtConfig.I_chmax);
            EEPROM.put(I_dchmax_ADDR, ProtConfig.I_dchmax);
            EEPROM.put(OCVP_ADDR, ProtConfig.OCVP);
            EEPROM.put(ODVP_ADDR, ProtConfig.ODVP);
            EEPROM.put(I_Bal_ADDR, ProtConfig.I_Bal);
        }
        
        EEPROM.get(SW_ADDR, MySW_Config.SW_Config);
        if (isnan(MySW_Config.SW_Config)) {
          MySW_Config.SW_Config = 9;
          EEPROM.put(SW_ADDR, MySW_Config.SW_Config);
        }


PORTB = MySW_Config.SW_Config;

Wire.begin(0x11); // Start I2C port 1 as slave with address 0x11
Wire.setClock(100000);
Wire.onRequest(onI2CRequest); // When master requests data, call onI2CRequest()
Wire.onReceive(onI2CReceive);

 previousTime = millis(); // Initialize time tracking for SoC estimation

// Serial.println("ADS1115 Measurement with Per-Channel Self-Calibration");
//     Serial.println("----------------------------------------");
//     for (int i = 0; i < 4; i++) {
//         Serial.print("Channel ");
//         Serial.print(i + 1);
//         Serial.print(": Slope = ");
//         Serial.print(calibrationSlope[i], 6);
//         Serial.print(", Offset = ");
//         Serial.println(calibrationOffset[i], 6);
//     }

}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(Pch, HIGH);    // Allow Charging
digitalWrite(Pdch, HIGH);   // Allow Discharging
digitalWrite(LED2, HIGH);
analogWrite(MBal, 0);      // Controls Balance Current (PWM)

// Process each channel
    for (int i = 0; i < 4; i++) {
        processChannel(i);
    }

    //printDivider();

    // Update SoC using Kalman Filter
    updateSoC(BatVoltage, BatCurrent);

    // Print the SoC status
    printSoCStatus(BatVoltage, BatCurrent, soc_estimate);

    // Print Vcc and Temp
    updateMeasurements();
//////////////////////////////////////////////////

  digitalWrite(LED2, HIGH);
  delay(500);
  digitalWrite(LED2, LOW);
  delay(500);

  //PORTB = MySW_Config.SW_Config;
  
}

////////////////////////////////////////////////////////////////////////
void serialEvent() {
  // Check if there's any data available
  if (Serial.available() > 0) {
    // Read the full command line (until newline)
    char command = (char)Serial.read();
    Serial.flush();
    //command.trim(); // Remove extra whitespace
    // Interpret commands
    if (command == '1') {
      selfCalibrate(0);
    } else if (command == '2') {
      selfCalibrate(1);
    } else if (command == '3') {
      selfCalibrate(2);
    } else if (command == '4') {
      selfCalibrate(3);
    } 
    if (command == 'b') {
      enterBatterySettingMode();
    }
  }
}
/////////////////////////////////////////////////////////////////////
// Process a single channel
void processChannel(int channel) {
    int16_t raw = ads.readRaw(channel);
    float voltage = ads.rawToVoltage(raw);

    // Serial.print("Channel ");
    // Serial.print(channel + 1);
    // Serial.print(": ");
    // Serial.print("Raw ADC: ");
    // Serial.print(raw);
    // Serial.print(", Voltage: ");
    // Serial.print(voltage, 6);

    if (channel == 0) { // Channel 1: Current measurement with ±2.5V reference
        float arduinoCurrent = calculateCurrent(voltage, TL431_VOLTAGE, NCS_GAIN, SHUNT_RESISTOR);
        float calibratedCurrent = calibrateCurrent(channel, arduinoCurrent);
        if (calibratedCurrent<0.001) {calibratedCurrent=0;}
        BatCurrent = calibratedCurrent;
        // Serial.print(", Arduino Current: ");
        // Serial.print(arduinoCurrent, 6);
        // Serial.print(" A, Calibrated Current: ");
        // Serial.print(calibratedCurrent, 6);
        // Serial.println(" A");
    } else if (channel == 1) { // Channel 2: Current measurement with 0V reference
        float current = calculateCurrent(voltage, 0.0, NCS_GAIN, 0.05); // Shunt = 50mΩ
        float calibratedCurrent = calibrateCurrent(channel, current);
        if (calibratedCurrent<0.001) {calibratedCurrent=0;}
        BalCurrent = calibratedCurrent;
        // Serial.print(", Current: ");
        // Serial.print(current, 6);
        // Serial.print(" A, Calibrated Current: ");
        // Serial.print(calibratedCurrent, 6);
        // Serial.println(" A");
    } else { // Channels 3 and 4: Voltage measurement
        float actualVoltage = calculateVoltage(voltage, DIVIDER_RATIO);
        float calibratedVoltage = calibrateCurrent(channel, actualVoltage); // Using same calibration logic
        if (channel == 2) {BatVoltage = calibratedVoltage;}
        if (channel == 3) {TerVoltage = calibratedVoltage;}
        // Serial.print(", Actual Voltage: ");
        // Serial.print(actualVoltage, 6);
        // Serial.print(" V, Calibrated Voltage: ");
        // Serial.print(calibratedVoltage, 6);
        // Serial.println(" V");
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
    while(Serial.available() > 0) { Serial.read(); }
    Serial.print("Starting Self-Calibration for Channel ");
    Serial.print(channel + 1);
    Serial.println("...");
    Serial.println("Provide known values for calibration. 20 points required.");

    float arduinoValues[20];
    float knownValues[20];
    int dataIndex = 0;

    while (dataIndex < 20) {

      // Ask the user for the known value
        Serial.print("Enter the known set-value: ");
        while (!Serial.available()); // Wait for user input
        String input = Serial.readStringUntil('\n');
        input.trim();

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
    float deltaSOC = (current * elapsedHours) / BatConfig.batteryCapacity * 100.0;
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
    if (voltage >= BatConfig.fullVoltage) return 100.0;
    if (voltage <= BatConfig.emptyVoltage) return 0.0;
    return (voltage - BatConfig.emptyVoltage) / (BatConfig.fullVoltage - BatConfig.emptyVoltage) * 100.0;
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
    EEPROM.get(FULL_VOLTAGE_ADDR,BatConfig.fullVoltage);
    EEPROM.get(EMPTY_VOLTAGE_ADDR,BatConfig.emptyVoltage);
    EEPROM.get(BAT_CAPACITY_ADDR,BatConfig.batteryCapacity);
    EEPROM.get(BAT_TYPE_ADDR,BatConfig.batteryType);

    Serial.println("Current Battery Setting ...");
    Serial.print("Battery Type: ");
    if (BatConfig.batteryType==1) Serial.println("LFP"); else Serial.println ("Li-ion");
    Serial.print("Full Voltage: ");
    Serial.println(BatConfig.fullVoltage, 2);
    Serial.print("Empty Voltage: ");
    Serial.println(BatConfig.emptyVoltage, 2);
    Serial.print("Capacity: ");
    Serial.println(BatConfig.batteryCapacity, 2);
    while(Serial.available() > 0) { Serial.read(); }
    Serial.println("Continue to change the battery settings: 1:Yes, 2:No");
    while (!Serial.available());
    int UserRes = Serial.parseInt();
    //Serial.readString();
    
    if (UserRes == 2) { return;}
    else {
    // Battery Type Selection
    Serial.println("Select Battery Type: ");
    Serial.println("1: LFP");
    Serial.println("2: Li-ion");
    while (!Serial.available());
    int batteryType = Serial.parseInt();
    Serial.readString();

    if (batteryType == 1) {
        //fullVoltage = 3.4;
        //emptyVoltage = 2.5;
        BatConfig.batteryType=1;
        Serial.println("LFP Battery Selected.");
    } else if (batteryType == 2) {
        //fullVoltage = 4.2;
        //emptyVoltage = 3.0;
        BatConfig.batteryType=2;
        Serial.println("Li-ion Battery Selected.");
    } else {
        Serial.println("Invalid input. Defaulting to LFP.");
     }

    // Custom Voltage Input
    Serial.println("Enter Fully Charged Voltage at 100% SoC:");
    while (!Serial.available());
    BatConfig.fullVoltage = Serial.parseFloat();
    Serial.readString();

    Serial.println("Enter Fully Discharged Voltage at 0% SoC:");
    while (!Serial.available());
    BatConfig.emptyVoltage = Serial.parseFloat();
    Serial.readString();

    Serial.println("Enter Battery Capacity:");
    while (!Serial.available());
    BatConfig.batteryCapacity = Serial.parseFloat();
    Serial.readString();

    // Save to EEPROM
    EEPROM.put(FULL_VOLTAGE_ADDR, BatConfig.fullVoltage);
    EEPROM.put(EMPTY_VOLTAGE_ADDR, BatConfig.emptyVoltage);
    EEPROM.put(BAT_CAPACITY_ADDR, BatConfig.batteryCapacity);
    EEPROM.put(BAT_TYPE_ADDR, BatConfig.batteryType);

    Serial.println("Settings Saved:");
    Serial.println("Exiting Battery Setting Mode...");
    }
}

///////////// Temp//////////////////////////////////////////////
float readVoltage(uint8_t pin) {
  // Read from the given ADC pin
  // Ensure ADC is configured as in original code (default AVcc reference)
  uint16_t raw = analogRead(pin);
  // We'll return raw ADC counts here and do conversions as needed
  return (float)raw;
}

float readTempC() {
  // Read thermistor voltage:
  float raw = readVoltage(THERMISTOR_PIN);
  // raw ADC to voltage: V = (raw/1023)*Vcc, but we don't know Vcc yet.
  // We'll first estimate Vcc using the TL431 reference.

  float vcc = estimateVcc();
  float voltage = (raw / ADC_MAX) * vcc;

  // Compute NTC resistance:
  // R_NTC = R_FIXED * (Vcc - Vout)/Vout
  float R_NTC = R_FIXED * (vcc - voltage) / voltage;

  // Temperature calculation:
  // 1/T = 1/T0 + (1/BETA)*ln(R_NTC/R0)
  float tempK = 1.0 / ((1.0/T0) + (1.0/BETA)*log(R_NTC/R0));
  float tempC = tempK - 273.15;
  return tempC;
}

float estimateVcc() {
  // Measure the known 2.5V reference at A3
  float refRaw = readVoltage(REF_PIN);

  // We know: 2.5V = (refRaw/1023)*Vcc
  // => Vcc = (2.5 * 1023)/refRaw
  float vcc = (2.5 * ADC_MAX) / refRaw;
  return vcc;
}

void updateMeasurements() {
  currentVcc = estimateVcc();
  currentTemperatureC = readTempC();
  Serial.print("VCC Value = ");
  Serial.print(currentVcc, 6);
  Serial.println(" V");
  Serial.print("Temp = ");
  Serial.print(currentTemperatureC, 6);
  Serial.println(" C");
}

////////////////////////////////////////

void sendFloat(float value) {
  union {
    float f;
    uint8_t b[4];
  } data;
  data.f = value;
  Wire.write(data.b, 4);
}

float readFloat() {
  union {
    float f;
    uint8_t b[4];
  } converter;

  for (int i = 0; i < 4; i++) {
    if (Wire.available()) {
      converter.b[i] = Wire.read();
    } else {
      converter.f = 0.0f; // fallback
    }
  }
  return converter.f;
}




