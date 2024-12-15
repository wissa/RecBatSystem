#include "ADS1115.h"

ADS1115::ADS1115(uint8_t address) : _address(address), _gain(ADS1115_PGA_2_048V),
                                    _dataRate(ADS1115_DATARATE_128SPS), _mode(ADS1115_MODE_SINGLESHOT) {}

void ADS1115::begin() {
    Wire1.begin();
}

void ADS1115::configure(uint16_t gain, uint16_t dataRate, uint16_t mode) {
    _gain = gain;
    _dataRate = dataRate;
    _mode = mode;
}

void ADS1115::setGain(uint16_t gain) {
    _gain = gain;
}

int16_t ADS1115::readRaw(uint8_t channel) {
    uint16_t config = ADS1115_OS_START // Start conversion
                    | _mode            // Single-shot or continuous mode
                    | _gain            // Gain setting
                    | _dataRate;       // Data rate setting

    // Set MUX for the specified channel
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

    // Wait for conversion to complete if in single-shot mode
    if (_mode == ADS1115_MODE_SINGLESHOT) {
        delay(8); // Adjust based on data rate
    }

    // Read the conversion result
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
    float lsb;
    switch (_gain) {
        case ADS1115_PGA_6_144V: lsb = 0.0001875; break;
        case ADS1115_PGA_4_096V: lsb = 0.000125; break;
        case ADS1115_PGA_2_048V: lsb = 0.0000625; break;
        case ADS1115_PGA_1_024V: lsb = 0.00003125; break;
        case ADS1115_PGA_0_512V: lsb = 0.000015625; break;
        case ADS1115_PGA_0_256V: lsb = 0.0000078125; break;
        default: lsb = 0.0000625; break;
    }
    return rawValue * lsb;
}

float ADS1115::rawToCurrent(int16_t rawValue, float referenceVoltage, float gain, float shuntResistor) {
    float voltage = rawToVoltage(rawValue);
    return (voltage - referenceVoltage) / (gain * shuntResistor);
}
