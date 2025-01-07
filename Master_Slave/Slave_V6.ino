#include "ADS1115.h"
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// Constants and Macros
const uint8_t BASE_I2C_ADDRESS = 0x10; // Base I2C Address
volatile uint8_t currentI2CAddress = BASE_I2C_ADDRESS; // Current I2C Address

const float TL431_VOLTAGE = 2.5f;
const float SHUNT_RESISTOR = 0.001f; // 1mΩ
const float NCS_GAIN = 50.0f;
const float DIVIDER_RATIO = 22.0f / 144.0f;
const unsigned long FAULT_LATCH_TIME = 10000UL; // 10 seconds
const int PWM_RESOLUTION = 255;
const float PWM_MAX = 1.0f;
const float PWM_MIN = 0.0f;
const float PWM_STEP_MAX = 0.05f;
const float Kp = 10.0f;
const float Ki = 1.0f;
const float INTEGRAL_LIMIT = 0.5f;
const int EEPROM_BASE_ADDR = 0;
const int SLOPE_OFFSET_SIZE = 8;

// EEPROM Addresses
enum EEPROMAddresses {
  FULL_VOLTAGE_ADDR = 36,
  EMPTY_VOLTAGE_ADDR = 40,
  BAT_CAPACITY_ADDR = 44,
  BAT_TYPE_ADDR = 48,
  I_CHMAX_ADDR = 56,
  I_DCHMAX_ADDR = 60,
  OCVP_ADDR = 64,
  ODVP_ADDR = 68,
  I_BAL_ADDR = 72,
  OVER_TEMP_ADDR = 76,
  SW_ADDR = 80 // to 90
};

// Fault Bits
enum FaultBits {
  FAULT_CHARGE_CURRENT = 0,
  FAULT_DISCHARGE_CURRENT,
  FAULT_OCVP,
  FAULT_ODVP,
  FAULT_LOW_VCC,
  FAULT_OVER_TEMP,
  FAULT_PRO_SW,
  NUM_FAULTS = 7
};

// Fault Masks
const byte CHARGE_FAULT_MASK = (1 << FAULT_CHARGE_CURRENT) | (1 << FAULT_OCVP) | (1 << FAULT_LOW_VCC) | (1 << FAULT_OVER_TEMP) | (1 << FAULT_PRO_SW);
const byte DISCHARGE_FAULT_MASK = (1 << FAULT_DISCHARGE_CURRENT) | (1 << FAULT_ODVP) | (1 << FAULT_LOW_VCC) | (1 << FAULT_OVER_TEMP) | (1 << FAULT_PRO_SW);

// Structures
struct BatteryConfig {
  uint8_t batteryType; // 1=LFP, 2=Li-ion
  float fullVoltage;
  float emptyVoltage;
  float batteryCapacity;
} BatConfig;

struct ProtectionConfig {
  float I_chmax;
  float I_dchmax;
  float OCVP;
  float ODVP;
  float I_Bal;
  float OverTemp;
} ProtConfig;

#define MAX_PATTERNS 10

struct Switch_Config {
  uint8_t SW_Config[MAX_PATTERNS];
  volatile uint8_t currentIndex = 0;
} MySW_Config;

// Globals
ADS1115 ads;

volatile bool addressChangeRequested = false;
volatile uint8_t newAddress = BASE_I2C_ADDRESS;
volatile uint8_t lastCommand = 0;

uint8_t FaultReg = 0;
unsigned long faultTimers[NUM_FAULTS] = {0};

float calibrationSlope[4] = {1.0f, 1.0f, 1.0f, 1.0f};
float calibrationOffset[4] = {0.0f, 0.0f, 0.0f, 0.0f};

float BatCurrent = 0.0f, BatVoltage = 0.0f, BalCurrent = 0.0f, TerVoltage = 0.0f;
float soc_estimate = 100.0f;
float Q = 0.001f, R = 0.1f, P = 1.0f, K = 0.0f;
unsigned long previousTime = 0;

float currentTemperatureC = 0.0f, currentVcc = 0.0f;

// Pin Definitions
const int SyncPin = 2; 
const int Pch = 5;   
const int Pdch = 6; 
const int MBal = 3; 
const int LED1 = 7; 
const int LED2 = 4; 

const int CSW_PINS[] = {8, 9, 10, 11, 12, 13, 20, 21};
const int ADD_PINS[] = {25, 26, 14, 15}; // Connected to PE2, PE3, PC0, PC1

#define THERMISTOR_PIN A2
#define REF_PIN A3
// Thermistor parameters
#define R_FIXED 10000.0    // Fixed resistor 10kΩ
#define R0      47000.0    // NTC resistance at 25°C
#define T0      298.15     // Kelvin (25°C)
#define BETA    4108.0     // Beta coefficient
#define ADC_MAX 1023.0

// Function Prototypes
void onI2CRequest();
void onI2CReceive(int numBytes);
void UpdateSW();
void processChannel(int channel);
float calculateCurrent(float measuredVoltage, float referenceVoltage, float gain, float shuntResistor);
float calibrateValue(int channel, float value);
float calculateVoltage(float measuredVoltage, float dividerGain);
void selfCalibrate(int channel);
void printDivider();
void updateSoC(float measuredVoltage, float current);
float voltageToSOC(float voltage);
void printSoCStatus(float voltage, float current, float soc);
void enterBatterySettingMode();
void enterProtectionSettingMode(); // New function for Serial 'q' command
float readVoltage(uint8_t pin);
float readTempC();
float estimateVcc();
void updateMeasurements();
void UpdateFaults();
void handleFaultClear(byte faultBit, unsigned long currentTime);
void sendFloat(float value);
float readFloat();
void updateBalancing();
uint8_t readI2CAddress();
void initializePins();
void loadCalibrationData();
void loadConfigurations();
void saveConfigurations();

// Timing Variables for Non-Blocking Operations
unsigned long previousLED2Toggle = 0;
unsigned long previousLED1Toggle = 0;
unsigned long previousPrintToggle = 0;
const unsigned long LED2Interval = 500; // 500 ms
const unsigned long LED1Interval = 1000; // 1000 ms
const unsigned long PrintInterval = 2000; // 2000 ms

// I2C Handlers
void onI2CRequest() {
  switch(lastCommand) {
    case 'A':
      sendFloat(BatVoltage);
      sendFloat(BatCurrent);
      sendFloat(soc_estimate);
      sendFloat(currentTemperatureC);
      sendFloat(currentVcc);
      sendFloat(BalCurrent);
      sendFloat(TerVoltage);
      Wire.write(FaultReg);
      lastCommand = 0;
      break;
    case 'B':
      Wire.write(BatConfig.batteryType);
      sendFloat(BatConfig.fullVoltage);
      sendFloat(BatConfig.emptyVoltage);
      sendFloat(BatConfig.batteryCapacity);
      lastCommand = 0;
      break;
    case 'P':
      sendFloat(ProtConfig.I_chmax);
      sendFloat(ProtConfig.I_dchmax);
      sendFloat(ProtConfig.OCVP);
      sendFloat(ProtConfig.ODVP);
      sendFloat(ProtConfig.I_Bal);
      sendFloat(ProtConfig.OverTemp);
      lastCommand = 0;
      break;
    default:
      break;
  }
}

void onI2CReceive(int numBytes) {
  if (numBytes < 1) return;
  lastCommand = Wire.read();

  if (lastCommand == 0xA0 && numBytes >= 2) { // Address Change Command
    newAddress = Wire.read();
    if (newAddress >= 0x10 && newAddress <= 0x7F) {
      addressChangeRequested = true;
      Serial.print(F("Received new I2C address: 0x"));
      if (newAddress < 16) Serial.print('0');
      Serial.println(newAddress, HEX);
    }
  } 
  else if (lastCommand == 'C' && numBytes >= 13) { // Battery Settings
    BatConfig.batteryType = Wire.read();
    BatConfig.fullVoltage = readFloat();
    BatConfig.emptyVoltage = readFloat();
    BatConfig.batteryCapacity = readFloat();
    saveConfigurations();
    lastCommand = 0;
    Serial.println(F("Battery settings updated via I2C."));
  } 
  else if (lastCommand == 'Q' && numBytes >= 24) { // Protection Settings (6 floats = 24 bytes)
    ProtConfig.I_chmax = readFloat();
    ProtConfig.I_dchmax = readFloat();
    ProtConfig.OCVP = readFloat();
    ProtConfig.ODVP = readFloat();
    ProtConfig.I_Bal = readFloat();
    ProtConfig.OverTemp = readFloat();
    saveConfigurations();
    lastCommand = 0;
    Serial.println(F("Protection settings updated via I2C."));
  } 
  else if (lastCommand == 'S' && numBytes >= 2) { // Switch Configuration
    for (int i = 0; i < MAX_PATTERNS; i++) {
      if (Wire.available()) {
        MySW_Config.SW_Config[i] = Wire.read();
      } else {
        MySW_Config.SW_Config[i] = 0xFF;
      }
      EEPROM.put(SW_ADDR + i, MySW_Config.SW_Config[i]);
      if (MySW_Config.SW_Config[i] == 0xFF) { break; }
    }
    // Update PORTB immediately
    PORTB = MySW_Config.SW_Config[0];
    lastCommand = 0;
    MySW_Config.currentIndex = 0;
    Serial.println(F("Switch configuration updated via I2C."));
  }
}

// Interrupt Service Routine for SyncPin
void UpdateSW() {
  PORTB = MySW_Config.SW_Config[MySW_Config.currentIndex];
  MySW_Config.currentIndex++;
  if (MySW_Config.currentIndex >= MAX_PATTERNS || MySW_Config.SW_Config[MySW_Config.currentIndex] == 0xFF) {
    MySW_Config.currentIndex = 0;
  }
}

// Setup Function
void setup() {
  initializePins();
  Serial.begin(9600);
  ads.begin();
  ads.configure(ADS1115_PGA_6_144V, ADS1115_DATARATE_128SPS, ADS1115_MODE_SINGLESHOT);
  
  loadCalibrationData();
  loadConfigurations();
  
  // Read DIP switches to set I2C address
  uint8_t dip_value = 0;
  for(int i = 0; i < 4; i++) {
    if(digitalRead(ADD_PINS[i]) == LOW) { // Assuming active LOW
      dip_value |= (1 << i);
    }
  }
  currentI2CAddress = BASE_I2C_ADDRESS + dip_value;
  Serial.print(F("I2C Address set to 0x"));
  if(currentI2CAddress < 16) Serial.print('0');
  Serial.println(currentI2CAddress, HEX);
  
  Wire.begin(currentI2CAddress);
  Wire.setClock(400000);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
  
  previousTime = millis();
  
  // Default settings
  digitalWrite(Pch, HIGH);
  digitalWrite(Pdch, HIGH);
  digitalWrite(LED2, LOW);
  analogWrite(MBal, 0);
}

// Main Loop
void loop() {
  
  // Handle Address Change Request
  if(addressChangeRequested) {
    Wire.begin(newAddress); // Re-initialize I²C with new address
    Wire.onRequest(onI2CRequest); // Re-register the request handler
    Wire.onReceive(onI2CReceive); // Re-register the receive handler
    currentI2CAddress = newAddress; // Update the current address
    Serial.print(F("*********************************************\n"));
    Serial.print(F("I2C Address changed to 0x"));
    if(newAddress < 16) Serial.print('0');
    Serial.println(newAddress, HEX);
    addressChangeRequested = false; // Reset the flag
    // Optionally, reset other I/O or states here if necessary
  }
  
  // Non-blocking LED2 toggling
  unsigned long currentMillis = millis();
  if (currentMillis - previousLED2Toggle >= LED2Interval && currentI2CAddress != BASE_I2C_ADDRESS) {
    previousLED2Toggle = currentMillis;
    digitalWrite(LED2, !digitalRead(LED2));
  }
  
  // Process channels
  for (int i = 0; i < 4; i++) {
    processChannel(i);
  }

  if (currentMillis - previousPrintToggle >= PrintInterval) {
    previousPrintToggle = currentMillis;
    printDivider();
    updateSoC(BatVoltage, BatCurrent);
    printSoCStatus(BatVoltage, BatCurrent, soc_estimate);
    updateMeasurements();
    UpdateFaults();
    updateBalancing();
  }
  
  // Non-blocking LED1 toggling based on FaultReg
  if (FaultReg != 0) {
    if (currentMillis - previousLED1Toggle >= LED1Interval) {
      previousLED1Toggle = currentMillis;
      digitalWrite(LED1, !digitalRead(LED1));
    }
  } else {
    digitalWrite(LED1, LOW);
  }

  // Avoid tight looping by allowing other processes (like I2C) to run
  // No delay() here
}

// Helper Functions

// Initialize Pins
void initializePins() {
  pinMode(SyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SyncPin), UpdateSW, FALLING);
  
  pinMode(Pch, OUTPUT);
  pinMode(Pdch, OUTPUT);
  pinMode(MBal, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  
  for(auto pin : CSW_PINS) pinMode(pin, OUTPUT);
  for(auto pin : ADD_PINS) pinMode(pin, INPUT_PULLUP); // Ensure pull-up resistors
}

// Load Calibration Data from EEPROM
void loadCalibrationData() {
  for (int i = 0; i < 4; i++) {
    EEPROM.get(EEPROM_BASE_ADDR + i * SLOPE_OFFSET_SIZE, calibrationSlope[i]);
    EEPROM.get(EEPROM_BASE_ADDR + i * SLOPE_OFFSET_SIZE + 4, calibrationOffset[i]);
    if (isnan(calibrationSlope[i]) || isnan(calibrationOffset[i])) {
      calibrationSlope[i] = 1.0f;
      calibrationOffset[i] = 0.0f;
      EEPROM.put(EEPROM_BASE_ADDR + i * SLOPE_OFFSET_SIZE, calibrationSlope[i]);
      EEPROM.put(EEPROM_BASE_ADDR + i * SLOPE_OFFSET_SIZE + 4, calibrationOffset[i]);
    }
  }
}

// Load Configurations from EEPROM
void loadConfigurations() {
  EEPROM.get(FULL_VOLTAGE_ADDR, BatConfig.fullVoltage);
  EEPROM.get(EMPTY_VOLTAGE_ADDR, BatConfig.emptyVoltage);
  EEPROM.get(BAT_CAPACITY_ADDR, BatConfig.batteryCapacity);
  EEPROM.get(BAT_TYPE_ADDR, BatConfig.batteryType);
  
  if (isnan(BatConfig.fullVoltage) || isnan(BatConfig.emptyVoltage) || isnan(BatConfig.batteryCapacity)) {
    BatConfig.fullVoltage = 3.4f;
    BatConfig.emptyVoltage = 2.5f;
    BatConfig.batteryCapacity = 14000.0f;
    BatConfig.batteryType = 1;
    saveConfigurations();
  }
  
  EEPROM.get(I_CHMAX_ADDR, ProtConfig.I_chmax);
  EEPROM.get(I_DCHMAX_ADDR, ProtConfig.I_dchmax);
  EEPROM.get(OCVP_ADDR, ProtConfig.OCVP);
  EEPROM.get(ODVP_ADDR, ProtConfig.ODVP);
  EEPROM.get(I_BAL_ADDR, ProtConfig.I_Bal);
  EEPROM.get(OVER_TEMP_ADDR, ProtConfig.OverTemp);
  
  if (isnan(ProtConfig.I_chmax) || isnan(ProtConfig.I_dchmax) || isnan(ProtConfig.OCVP) || 
      isnan(ProtConfig.ODVP) || isnan(ProtConfig.I_Bal) || isnan(ProtConfig.OverTemp)) {
    ProtConfig.I_chmax = 1.0f;
    ProtConfig.I_dchmax = 2.0f;
    ProtConfig.OCVP = 3.55f;
    ProtConfig.ODVP = 2.6f;
    ProtConfig.I_Bal = 0.3f;
    ProtConfig.OverTemp = 90.0f;
    saveConfigurations();
  }
  
  for (int i = 0; i < MAX_PATTERNS; i++) {
    EEPROM.get(SW_ADDR + i, MySW_Config.SW_Config[i]);
    if (MySW_Config.SW_Config[i] == 0xFF) {
      MySW_Config.SW_Config[i] = 0xFF;
      EEPROM.put(SW_ADDR + i, MySW_Config.SW_Config[i]);
    }
  }
    
  PORTB = 0x00; // Initialize PORTB to 0
}

// Save Configurations to EEPROM
void saveConfigurations() {
  EEPROM.put(FULL_VOLTAGE_ADDR, BatConfig.fullVoltage);
  EEPROM.put(EMPTY_VOLTAGE_ADDR, BatConfig.emptyVoltage);
  EEPROM.put(BAT_CAPACITY_ADDR, BatConfig.batteryCapacity);
  EEPROM.put(BAT_TYPE_ADDR, BatConfig.batteryType);
  
  EEPROM.put(I_CHMAX_ADDR, ProtConfig.I_chmax);
  EEPROM.put(I_DCHMAX_ADDR, ProtConfig.I_dchmax);
  EEPROM.put(OCVP_ADDR, ProtConfig.OCVP);
  EEPROM.put(ODVP_ADDR, ProtConfig.ODVP);
  EEPROM.put(I_BAL_ADDR, ProtConfig.I_Bal);
  EEPROM.put(OVER_TEMP_ADDR, ProtConfig.OverTemp);
}

// Send Float via I2C
void sendFloat(float value) {
  union {
    float f;
    uint8_t b[4];
  } data;
  data.f = value;
  Wire.write(data.b, 4);
}

// Read Float from I2C
float readFloat() {
  union {
    float f;
    uint8_t b[4];
  } converter = {0};
  
  for (int i = 0; i < 4; i++) {
    if (Wire.available()) {
      converter.b[i] = Wire.read();
    }
  }
  return converter.f;
}

// Process ADC Channels
void processChannel(int channel) {
  int16_t raw = ads.readRaw(channel);
  float voltage = ads.rawToVoltage(raw);
  
  switch(channel) {
    case 0: { // Current measurement with ±2.5V reference
      float arduinoCurrent = calculateCurrent(voltage, TL431_VOLTAGE, NCS_GAIN, SHUNT_RESISTOR);
      float calibratedCurrent = calibrateValue(channel, arduinoCurrent);
      BatCurrent = (calibratedCurrent < 0.001f && calibratedCurrent > -0.001f) ? 0.0f : calibratedCurrent;
      break;
    }
    case 1: { // Current measurement with 0V reference
      float current = calculateCurrent(voltage, 0.0f, NCS_GAIN, 0.05f);
      float calibratedCurrent = calibrateValue(channel, current);
      BalCurrent = (calibratedCurrent < 0.001f) ? 0.0f : calibratedCurrent;
      break;
    }
    case 2: // Voltage measurement
      BatVoltage = calibrateValue(channel, calculateVoltage(voltage, DIVIDER_RATIO));
      break;
    case 3: // Terminal Voltage
      TerVoltage = calibrateValue(channel, calculateVoltage(voltage, DIVIDER_RATIO));
      break;
    default:
      break;
  }
}

// Calculate Current
float calculateCurrent(float measuredVoltage, float referenceVoltage, float gain, float shuntResistor) {
  return (measuredVoltage - referenceVoltage) / (gain * shuntResistor);
}

// Calibrate Value
float calibrateValue(int channel, float value) {
  return (calibrationSlope[channel] * value) + calibrationOffset[channel];
}

// Calculate Voltage
float calculateVoltage(float measuredVoltage, float dividerGain) {
  return measuredVoltage / dividerGain;
}

// Self Calibration Function (Unused in Current Context)
void selfCalibrate(int channel) {
  // Implementation remains unchanged or can be adapted as needed
}

// Print Divider
void printDivider() {
  Serial.println(F("----------------------------------------"));
}

// Update State of Charge (SoC)
void updateSoC(float measuredVoltage, float current) {
  unsigned long currentTime = millis();
  float elapsedHours = (currentTime - previousTime) / 3600000.0f; // ms to hours
  previousTime = currentTime;
  
  // Coulomb Counting
  float deltaSOC = (current * elapsedHours) / BatConfig.batteryCapacity * 100.0f;
  soc_estimate -= deltaSOC;
  
  // Kalman Filter
  P += Q;
  K = P / (P + R);
  float soc_from_voltage = voltageToSOC(measuredVoltage);
  soc_estimate += K * (soc_from_voltage - soc_estimate);
  P *= (1.0f - K);
  
  soc_estimate = constrain(soc_estimate, 0.0f, 100.0f);
}

// Voltage to SoC Conversion
float voltageToSOC(float voltage) {
  if (voltage >= BatConfig.fullVoltage) return 100.0f;
  if (voltage <= BatConfig.emptyVoltage) return 0.0f;
  return (voltage - BatConfig.emptyVoltage) / (BatConfig.fullVoltage - BatConfig.emptyVoltage) * 100.0f;
}

// Print SoC Status
void printSoCStatus(float voltage, float current, float soc) {
  Serial.print(F("Battery Voltage: "));
  Serial.print(voltage, 6);
  Serial.print(F(" V, Battery Current: "));
  Serial.print(current, 6);
  Serial.print(F(" A, SoC: "));
  Serial.print(soc, 1);
  Serial.println(F(" %"));
}

// Enter Battery Setting Mode (Unused in Current Context)
void enterBatterySettingMode() {
  Serial.println(F("Entering Battery Setting Mode..."));
  Serial.println(F("Current Battery Settings:"));
  Serial.print(F("Battery Type: "));
  Serial.println(BatConfig.batteryType == 1 ? F("LFP") : F("Li-ion"));
  Serial.print(F("Full Voltage: "));
  Serial.println(BatConfig.fullVoltage, 2);
  Serial.print(F("Empty Voltage: "));
  Serial.println(BatConfig.emptyVoltage, 2);
  Serial.print(F("Capacity: "));
  Serial.println(BatConfig.batteryCapacity, 2);
  
  while(Serial.available() > 0) { Serial.read(); }
  Serial.println(F("Continue to change the battery settings? 1:Yes, 2:No"));
  
  while (!Serial.available());
  int UserRes = Serial.parseInt();
  Serial.read(); // Clear the buffer
  
  if (UserRes != 1) { 
    Serial.println(F("Exiting Battery Setting Mode..."));
    return; 
  }
  
  // Battery Type Selection
  Serial.println(F("Select Battery Type: "));
  Serial.println(F("1: LFP"));
  Serial.println(F("2: Li-ion"));
  
  while (!Serial.available());
  int batteryType = Serial.parseInt();
  Serial.read(); // Clear the buffer
  
  BatConfig.batteryType = (batteryType == 2) ? 2 : 1;
  Serial.println(BatConfig.batteryType == 1 ? F("LFP Battery Selected.") : F("Li-ion Battery Selected."));
  
  // Custom Voltage Input
  Serial.println(F("Enter Fully Charged Voltage at 100% SoC:"));
  while (!Serial.available());
  BatConfig.fullVoltage = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  Serial.println(F("Enter Fully Discharged Voltage at 0% SoC:"));
  while (!Serial.available());
  BatConfig.emptyVoltage = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  Serial.println(F("Enter Battery Capacity:"));
  while (!Serial.available());
  BatConfig.batteryCapacity = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  saveConfigurations();
  
  Serial.println(F("Settings Saved:"));
  Serial.println(F("Exiting Battery Setting Mode..."));
}

// Enter Protection Setting Mode via Serial 'q' Command
void enterProtectionSettingMode() {
  Serial.println(F("Entering Protection Setting Mode..."));
  Serial.println(F("Current Protection Settings:"));
  Serial.print(F("I_chmax: "));
  Serial.println(ProtConfig.I_chmax, 2);
  Serial.print(F("I_dchmax: "));
  Serial.println(ProtConfig.I_dchmax, 2);
  Serial.print(F("OCVP: "));
  Serial.println(ProtConfig.OCVP, 2);
  Serial.print(F("ODVP: "));
  Serial.println(ProtConfig.ODVP, 2);
  Serial.print(F("I_Bal: "));
  Serial.println(ProtConfig.I_Bal, 2);
  Serial.print(F("OverTemp: "));
  Serial.println(ProtConfig.OverTemp, 2);
  
  while(Serial.available() > 0) { Serial.read(); }
  Serial.println(F("Continue to change the protection settings? 1:Yes, 2:No"));
  
  while (!Serial.available());
  int UserRes = Serial.parseInt();
  Serial.read(); // Clear the buffer
  
  if (UserRes != 1) { 
    Serial.println(F("Exiting Protection Setting Mode..."));
    return; 
  }
  
  // Protection Settings Input
  Serial.println(F("Enter I_chmax (Max Charging Current):"));
  while (!Serial.available());
  ProtConfig.I_chmax = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  Serial.println(F("Enter I_dchmax (Max Discharging Current):"));
  while (!Serial.available());
  ProtConfig.I_dchmax = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  Serial.println(F("Enter OCVP (Over Charge Voltage Protection):"));
  while (!Serial.available());
  ProtConfig.OCVP = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  Serial.println(F("Enter ODVP (Over Discharge Voltage Protection):"));
  while (!Serial.available());
  ProtConfig.ODVP = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  Serial.println(F("Enter I_Bal (Balance Current):"));
  while (!Serial.available());
  ProtConfig.I_Bal = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  Serial.println(F("Enter OverTemp (Over Temperature Threshold):"));
  while (!Serial.available());
  ProtConfig.OverTemp = Serial.parseFloat();
  Serial.read(); // Clear the buffer
  
  saveConfigurations();
  
  Serial.println(F("Protection Settings Saved."));
  Serial.println(F("Exiting Protection Setting Mode..."));
}

// Read Voltage from a Pin (Unused in Current Context)
float readVoltage(uint8_t pin) {
  return (float)analogRead(pin);
}

// Read Temperature in Celsius
float readTempC() {
  float raw = readVoltage(THERMISTOR_PIN);
  float vcc = estimateVcc();
  float voltage = (raw / ADC_MAX) * vcc;
  float R_NTC = R_FIXED * (vcc - voltage) / voltage;
  float tempK = 1.0f / ((1.0f / T0) + (1.0f / BETA) * log(R_NTC / R0));
  return tempK - 273.15f;
}

// Estimate Vcc
float estimateVcc() {
  float refRaw = readVoltage(REF_PIN);
  return (TL431_VOLTAGE * ADC_MAX) / refRaw;
}

// Update Measurements
void updateMeasurements() {
  currentVcc = estimateVcc();
  currentTemperatureC = readTempC();
  Serial.print(F("VCC: "));
  Serial.print(currentVcc, 6);
  Serial.println(F(" V"));
  Serial.print(F("Temp: "));
  Serial.print(currentTemperatureC, 6);
  Serial.println(F(" C"));
  Serial.print(F("Balance Current: "));
  Serial.print(BalCurrent, 6);
  Serial.println(F(" A"));
  Serial.print(F("Terminal Voltage: "));
  Serial.print(TerVoltage, 6);
  Serial.println(F(" V"));
}

// Update Faults
void UpdateFaults() {
  unsigned long currentTime = millis();
  
  // Charging Current Fault
  if (BatCurrent < -ProtConfig.I_chmax) {
    FaultReg |= (1 << FAULT_CHARGE_CURRENT);
    faultTimers[FAULT_CHARGE_CURRENT] = currentTime;
  } else {
    handleFaultClear(FAULT_CHARGE_CURRENT, currentTime);
  }
  
  // Discharging Current Fault
  if (BatCurrent > ProtConfig.I_dchmax) {
    FaultReg |= (1 << FAULT_DISCHARGE_CURRENT);
    faultTimers[FAULT_DISCHARGE_CURRENT] = currentTime;
  } else {
    handleFaultClear(FAULT_DISCHARGE_CURRENT, currentTime);
  }
  
  // Over-Voltage Charging Fault
  if (BatVoltage > ProtConfig.OCVP) {
    FaultReg |= (1 << FAULT_OCVP);
    faultTimers[FAULT_OCVP] = currentTime;
  } else {
    handleFaultClear(FAULT_OCVP, currentTime);
  }
  
  // Over-Voltage Discharging Fault
  if (BatVoltage < ProtConfig.ODVP) {
    FaultReg |= (1 << FAULT_ODVP);
    faultTimers[FAULT_ODVP] = currentTime;
  } else {
    handleFaultClear(FAULT_ODVP, currentTime);
  }
  
  // Low Vcc Fault
  if (currentVcc < 4.2f) {
    FaultReg |= (1 << FAULT_LOW_VCC);
    faultTimers[FAULT_LOW_VCC] = currentTime;
  } else {
    handleFaultClear(FAULT_LOW_VCC, currentTime);
  }
  
  // Over Temperature Fault
  if (currentTemperatureC > ProtConfig.OverTemp) {
    FaultReg |= (1 << FAULT_OVER_TEMP);
    faultTimers[FAULT_OVER_TEMP] = currentTime;
  } else {
    handleFaultClear(FAULT_OVER_TEMP, currentTime);
  }
  
  // Protection Switches Malfunctioning Fault
  if ((BatVoltage - TerVoltage) > 0.1f && digitalRead(Pch) && digitalRead(Pdch)) {
    FaultReg |= (1 << FAULT_PRO_SW);
    faultTimers[FAULT_PRO_SW] = currentTime;
  } else {
    handleFaultClear(FAULT_PRO_SW, currentTime);
  }
  
  // FaultReg Debug
  Serial.print(F("FaultReg: "));
  Serial.println(FaultReg, BIN);
  
  // Control Charging
  if (FaultReg & CHARGE_FAULT_MASK) {
    digitalWrite(Pch, LOW);
    Serial.println(F("Disable Charging"));
  } else {
    digitalWrite(Pch, HIGH);
    Serial.println(F("Enable Charging"));
  }
  
  // Control Discharging
  if (FaultReg & DISCHARGE_FAULT_MASK) {
    digitalWrite(Pdch, LOW);
    Serial.println(F("Disable Discharging"));
  } else {
    digitalWrite(Pdch, HIGH);
    Serial.println(F("Enable Discharging"));
  }
}

// Handle Fault Clearing
void handleFaultClear(byte faultBit, unsigned long currentTime) {
  if (FaultReg & (1 << faultBit)) {
    if ((currentTime - faultTimers[faultBit]) >= FAULT_LATCH_TIME) {
      FaultReg &= ~(1 << faultBit);
      faultTimers[faultBit] = 0;
      Serial.print(F("Cleared Fault Bit: "));
      Serial.println(faultBit);
    }
  }
}

// Update Balancing
void updateBalancing() {
  static float currentDutyCycle = 0.0f;
  static float integral = 0.0f;
  
  if (ProtConfig.I_Bal <= 0.0f || BatVoltage <= ProtConfig.OCVP) {
    analogWrite(MBal, 0);
    currentDutyCycle = 0.0f;
    integral = 0.0f;
    return;
  }
  
  float error = ProtConfig.I_Bal - BalCurrent;
  integral += error * 0.1f; // Assuming this function is called every 100ms
  
  // Clamp integral
  if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
  if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
  
  float adjustment = (Kp * error) + (Ki * integral);
  float newDutyCycle = currentDutyCycle + adjustment;
  
  // Clamp duty cycle
  newDutyCycle = constrain(newDutyCycle, PWM_MIN, PWM_MAX);
  
  // Limit step change
  if (newDutyCycle > currentDutyCycle + PWM_STEP_MAX)
    newDutyCycle = currentDutyCycle + PWM_STEP_MAX;
  if (newDutyCycle < currentDutyCycle - PWM_STEP_MAX)
    newDutyCycle = currentDutyCycle - PWM_STEP_MAX;
  
  analogWrite(MBal, (int)(newDutyCycle * PWM_RESOLUTION));
  currentDutyCycle = newDutyCycle;
}

// Serial Event Handler
void serialEvent() {
  while (Serial.available()) {
    char command = (char)Serial.read();
    switch(command) {
      case '1':
      case '2':
      case '3':
      case '4':
        selfCalibrate(command - '1');
        break;
      case 'b':
        enterBatterySettingMode();
        break;
      case 'q': // New Serial Command for Protection Settings
        enterProtectionSettingMode();
        break;
      default:
        break;
    }
  }
}
