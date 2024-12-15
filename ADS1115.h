#ifndef ADS1115_H
#define ADS1115_H

#include <Wire1.h>

// Default I2C address for ADS1115
#define ADS1115_ADDRESS 0x48

// Register Addresses
#define ADS1115_REG_CONVERT   0x00 // Conversion register
#define ADS1115_REG_CONFIG    0x01 // Configuration register

// Gain Settings (PGA)
#define ADS1115_PGA_6_144V    0x0000 // ±6.144V range (187.5 µV/LSB)
#define ADS1115_PGA_4_096V    0x0200 // ±4.096V range (125 µV/LSB)
#define ADS1115_PGA_2_048V    0x0400 // ±2.048V range (62.5 µV/LSB) - Default
#define ADS1115_PGA_1_024V    0x0600 // ±1.024V range (31.25 µV/LSB)
#define ADS1115_PGA_0_512V    0x0800 // ±0.512V range (15.625 µV/LSB)
#define ADS1115_PGA_0_256V    0x0A00 // ±0.256V range (7.8125 µV/LSB)

// Data Rate Settings
#define ADS1115_DATARATE_8SPS    0x0000 // 8 samples per second
#define ADS1115_DATARATE_16SPS   0x0020 // 16 samples per second
#define ADS1115_DATARATE_32SPS   0x0040 // 32 samples per second
#define ADS1115_DATARATE_64SPS   0x0060 // 64 samples per second
#define ADS1115_DATARATE_128SPS  0x0080 // 128 samples per second (default)
#define ADS1115_DATARATE_250SPS  0x00A0 // 250 samples per second
#define ADS1115_DATARATE_475SPS  0x00C0 // 475 samples per second
#define ADS1115_DATARATE_860SPS  0x00E0 // 860 samples per second (fastest)

// Mode Settings
#define ADS1115_MODE_CONTINUOUS  0x0000 // Continuous conversion mode
#define ADS1115_MODE_SINGLESHOT  0x0100 // Single-shot mode (default)

// MUX Settings for Single-Ended Channels
#define ADS1115_MUX_CH0 0x4000
#define ADS1115_MUX_CH1 0x5000
#define ADS1115_MUX_CH2 0x6000
#define ADS1115_MUX_CH3 0x7000

// Operational Status (OS) Bit
#define ADS1115_OS_START 0x8000 // Start single-shot conversion

class ADS1115 {
public:
    ADS1115(uint8_t address = ADS1115_ADDRESS);

    void begin();
    void configure(uint16_t gain = ADS1115_PGA_4_096V,
                   uint16_t dataRate = ADS1115_DATARATE_128SPS,
                   uint16_t mode = ADS1115_MODE_SINGLESHOT);
    void setGain(uint16_t gain);
    int16_t readRaw(uint8_t channel);
    float rawToVoltage(int16_t rawValue);
    float rawToCurrent(int16_t rawValue, float referenceVoltage, float gain, float shuntResistor);

private:
    uint8_t _address;
    uint16_t _gain;
    uint16_t _dataRate;
    uint16_t _mode;
    void configureChannel(uint8_t channel);
};

#endif
