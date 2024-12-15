#ifndef ADS1115_H
#define ADS1115_H

#include <Wire1.h>

/**
 * @brief Library for controlling the ADS1115 using Wire1 (ATmega328PB).
 * 
 * Features:
 * - Read single-ended channels (CH0 to CH3).
 * - Convert raw ADC readings into current or voltage.
 * - Adjustable gain and reference voltage.
 */

#define ADS1115_ADDRESS 0x48 // Default I2C address for ADS1115

// Register Addresses
#define ADS1115_REG_CONVERT   0x00 // Conversion register
#define ADS1115_REG_CONFIG    0x01 // Configuration register

// Default Configurations
#define ADS1115_PGA_2_048V    0x0400 // ±2.048V range (gain)
#define ADS1115_DATARATE_1600SPS 0x0080 // 1600 samples per second
#define ADS1115_MODE_SINGLESHOT 0x0100 // Single-shot mode
#define ADS1115_OS_START      0x8000 // Start a conversion

// MUX settings for single-ended channels
#define ADS1115_MUX_CH0 0x4000
#define ADS1115_MUX_CH1 0x5000
#define ADS1115_MUX_CH2 0x6000
#define ADS1115_MUX_CH3 0x7000

// Conversion delay (in milliseconds)
#define ADS1115_CONVERSION_DELAY 8 // Minimum time for conversion to complete

class ADS1115 {
public:
    /**
     * @brief Constructor to initialize the ADS1115.
     * @param address I2C address of the ADS1115 (default: 0x48).
     */
    ADS1115(uint8_t address = ADS1115_ADDRESS);

    /**
     * @brief Initializes the Wire1 library and sets up the device.
     */
    void begin();

    /**
     * @brief Reads raw ADC value from the specified channel.
     * @param channel Channel number (0-3).
     * @return Raw ADC value (16-bit signed).
     */
    int16_t readRaw(uint8_t channel);

    /**
     * @brief Converts raw ADC value to voltage.
     * @param rawValue Raw ADC value.
     * @return Voltage in volts.
     */
    float rawToVoltage(int16_t rawValue);

    /**
     * @brief Converts raw ADC value to current based on shunt resistor and gain.
     * @param rawValue Raw ADC value.
     * @param referenceVoltage Reference voltage for the measurement (e.g., 2.5V for bidirectional currents).
     * @param gain Gain of the current sensor.
     * @param shuntResistor Shunt resistor value in ohms.
     * @return Current in amperes.
     */
    float rawToCurrent(int16_t rawValue, float referenceVoltage, float gain, float shuntResistor);

private:
    uint8_t _address; // I2C address
    void configureChannel(uint8_t channel); // Configures the ADS1115 for the specified channel
};

#endif
