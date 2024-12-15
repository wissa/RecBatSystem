#include "ADS1115.h"

ADS1115::ADS1115(uint8_t address) : _address(address) {}

void ADS1115::begin() {
    Wire1.begin(); // Initialize Wire1
}

int16_t ADS1115::readRaw(uint8_t channel) {
    uint16_t config = ADS1115_OS_START           // Start a single conversion
                    | ADS1115_MODE_SINGLESHOT    // Single-shot mode
                    | ADS1115_PGA_2_048V         // ±2.048V range
                    | ADS1115_DATARATE_1600SPS;  // 1600 samples per second

    // Set the MUX for the specified channel
    switch (channel) {
        case 0: config |= ADS1115_MUX_CH0; break;
        case 1: config |= ADS1115_MUX_CH1; break;
        case 2: config |= ADS1115_MUX_CH2; break;
        case 3: config |= ADS1115_MUX_CH3; break;
        default: return 0; // Invalid channel
    }

    // Write configuration to ADS1115
    Wire1.beginTransmission(_address);
    Wire1.write(ADS1115_REG_CONFIG);
    Wire1.write((uint8_t)(config >> 8)); // High byte
    Wire1.write((uint8_t)(config & 0xFF)); // Low byte
    Wire1.endTransmission();

    // Wait for conversion to complete
    delay(ADS1115_CONVERSION_DELAY);

    // Read conversion result
    Wire1.beginTransmission(_address);
    Wire1.write(ADS1115_REG_CONVERT);
    Wire1.endTransmission();

    Wire1.requestFrom(_address, (uint8_t)2);
    if (Wire1.available() >= 2) {
        uint16_t highByte = Wire1.read();
        uint16_t lowByte = Wire1.read();
        return (int16_t)((highByte << 8) | lowByte); // Combine bytes
    }

    return 0; // No data available
}

float ADS1115::rawToVoltage(int16_t rawValue) {
    return rawValue * 0.0000625; // 62.5 µV per count
}

float ADS1115::rawToCurrent(int16_t rawValue, float referenceVoltage, float gain, float shuntResistor) {
    float voltage = rawToVoltage(rawValue); // Convert to voltage
    return (voltage - referenceVoltage) / (gain * shuntResistor); // Calculate current
}
