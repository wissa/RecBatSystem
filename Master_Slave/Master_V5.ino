#include <Wire.h>

// I2C Addresses and other defines
#define DEFAULT_SLAVE_ADDRESS 0x10
#define SLAVE_ADDRESS_START    0x11
#define SLAVE_ADDRESS_END      0x7F
#define COMMAND_SET_ADDRESS    0xA0

// Sync Pin
#define SYNC_PIN 2  // Pin 2 for Sync Signal

// Timing Intervals (in milliseconds)
const unsigned long ADDRESS_ASSIGN_INTERVAL = 10000; // 10 seconds
const unsigned long DATA_COLLECTION_INTERVAL = 5000; // 5 seconds
const unsigned long SLAVE_TIMEOUT = 15000;            // 15 seconds

// Command Buffer Size
#define SERIAL_BUFFER_SIZE 64  // Increased to accommodate additional parameters

// Maximum number of slaves
#define MAX_SLAVES 50  // Adjust based on your application's needs

// Switch Configuration (Declared Globally)
static const uint8_t SW_Config[] = {0b10101011, 0b10101010, 0b00101001, 0b00101010, 0xFF}; // Example Switch Configuration 

// Slave Structure
struct Slave {
  uint8_t address;
  unsigned long lastResponse; // Timestamp of the last successful communication
};

// Slave Management
Slave slaves[MAX_SLAVES];
uint8_t slaveCount = 0;
uint8_t nextAddress = SLAVE_ADDRESS_START;

// Non-blocking Timing Variables
unsigned long previousAddressAssign = 0;
unsigned long previousDataCollection = 0;

// Serial Command Buffer
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialIndex = 0;

// Function Prototypes
void assignAddresses();
void checkForNewSlaves();
void requestSlaveData(char command);
void requestSlaveDataSingle(char command, uint8_t address);
void setSlaveData(char command, uint8_t address, const char* params);
void sendFloat(float value);
float readFloat(const byte* buffer, int index);
void handleSerialInput();
bool isAddressInUse(uint8_t address);
void removeSlave(uint8_t index);

// ISR for Timer1 Compare Match A
ISR(TIMER1_COMPA_vect) {
  // Toggle SYNC_PIN using direct port manipulation for speed
  PORTD ^= (1 << PD2); // Pin 2 on Arduino Uno corresponds to PD2
}

void setup() {
  // Initialize Pins
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW); // Ensure SYNC_PIN starts LOW
  pinMode(13, OUTPUT);         // Optional: Built-in LED for debugging
  digitalWrite(13, LOW);       // Ensure LED starts OFF

  // Initialize Serial and I2C
  Serial.begin(9600);
  Wire.begin(); // Master mode
  Wire.setClock(100000); // 100 kHz I2C clock
  delay(1000); // Allow time for I2C devices to initialize
  Serial.println("Master ready.");

  // Timer1 Configuration for SYNC_PIN toggling
  noInterrupts();           // Disable all interrupts

  TCCR1A = 0;               // Set entire TCCR1A register to 0
  TCCR1B = 0;               // Same for TCCR1B
  TCNT1  = 0;               // Initialize counter value to 0

  // Set CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);

  // Set prescaler to 8
  TCCR1B |= (1 << CS11); // Prescaler bits CS12:0 = 010 for prescaler 8

  // Set compare match register for 40 kHz (25 µs toggle interval)
  OCR1A = 99; // (16,000,000 / (8 * 40,000)) - 1 = 49

  // Enable Timer1 Compare Interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();             // Enable all interrupts

  // Initial Address Assignment
  assignAddresses();
}

void loop() {
  unsigned long currentMillis = millis();

  // Handle Serial Input
  handleSerialInput();

  // Periodically Assign Addresses and Check for New Slaves
  if (currentMillis - previousAddressAssign >= ADDRESS_ASSIGN_INTERVAL) {
    previousAddressAssign += ADDRESS_ASSIGN_INTERVAL;
    assignAddresses();
    checkForNewSlaves();
  }

  // Periodically Collect Data from Slaves
  if (currentMillis - previousDataCollection >= DATA_COLLECTION_INTERVAL) {
    previousDataCollection += DATA_COLLECTION_INTERVAL;
    requestSlaveData('A'); // Command 'A' for data collection
  }

  // Check for Slaves that have Timed Out
  for (uint8_t i = 0; i < slaveCount; i++) {
    if (millis() - slaves[i].lastResponse > SLAVE_TIMEOUT) {
      Serial.print("Slave 0x");
      if (slaves[i].address < 16) Serial.print("0");
      Serial.print(slaves[i].address, HEX);
      Serial.println(" timed out. Removing from active list.");
      removeSlave(i);
      i--; // Adjust index after removal
    }
  }

  // Other non-blocking tasks can be added here
}

///////////////////////////////////////
// Address Assignment and Slave Detection
///////////////////////////////////////
void assignAddresses() {
  if (nextAddress > SLAVE_ADDRESS_END || slaveCount >= MAX_SLAVES) {
    Serial.println("Address assignment completed or max slaves reached.");
    return;
  }

  // Attempt to assign nextAddress to a new slave
  Wire.beginTransmission(DEFAULT_SLAVE_ADDRESS);
  Wire.write(COMMAND_SET_ADDRESS); // Command byte
  Wire.write(nextAddress);         // New address byte
  uint8_t error = Wire.endTransmission(true);

  if (error == 0) {
    Serial.print("Assigned new address 0x");
    if (nextAddress < 16) Serial.print("0");
    Serial.println(nextAddress, HEX);
    if (slaveCount < MAX_SLAVES) {
      slaves[slaveCount].address = nextAddress;
      slaves[slaveCount].lastResponse = millis(); // Initialize lastResponse
      slaveCount++;
    }
    nextAddress++;
  } else {
    Serial.println("No more slaves responding at default address.");
    nextAddress = SLAVE_ADDRESS_START; // Reset to start checking
  }
}

void checkForNewSlaves() {
  // Scan I2C bus for new slaves within address range
  Serial.println("Scanning for new slaves...");
  for (uint8_t addr = SLAVE_ADDRESS_START; addr <= SLAVE_ADDRESS_END; addr++) {
    if (!isAddressInUse(addr)) {
      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();
      if (error == 0) {
        if (slaveCount < MAX_SLAVES) {
          slaves[slaveCount].address = addr;
          slaves[slaveCount].lastResponse = millis();
          Serial.print("Detected new slave at address 0x");
          if (addr < 16) Serial.print("0");
          Serial.println(addr, HEX);
          slaveCount++;
        } else {
          Serial.println("Maximum number of slaves reached.");
          break;
        }
      }
    }
  }
}

bool isAddressInUse(uint8_t address) {
  for (uint8_t i = 0; i < slaveCount; i++) {
    if (slaves[i].address == address) {
      return true;
    }
  }
  return false;
}

void removeSlave(uint8_t index) {
  if (index >= slaveCount) return;

  // Shift the remaining slaves left to fill the gap
  for (uint8_t i = index; i < slaveCount - 1; i++) {
    slaves[i] = slaves[i + 1];
  }
  slaveCount--;
}

///////////////////////////////////////
// Data Collection and Command Handling
///////////////////////////////////////
void requestSlaveData(char command) {
  // 'A' command: Request data from all slaves
  if (command != 'A') return; // Safety check

  for (uint8_t i = 0; i < slaveCount; i++) {
    Wire.beginTransmission(slaves[i].address);
    Wire.write(command); // Send command 'A'
    uint8_t error = Wire.endTransmission(false); // Send without releasing the bus

    if (error == 0) {
      // Determine expected data size based on command
      uint8_t expectedSize = 0;
      switch (command) {
        case 'A':
          expectedSize = (4 * 7) + 1; // 29 bytes
          break;
        // Add cases for other global commands if needed
        default:
          expectedSize = 0;
          break;
      }

      if (expectedSize == 0) {
        Serial.println("Unknown command for data request.");
        continue;
      }

      Wire.requestFrom(slaves[i].address, expectedSize);
      byte buffer[32]; // Adjust size if needed
      uint8_t idx = 0;
      while (Wire.available() && idx < expectedSize) {
        buffer[idx++] = Wire.read();
      }

      if (idx == expectedSize) {
        // Update lastResponse timestamp
        slaves[i].lastResponse = millis();

        Serial.print("Data from slave 0x");
        if (slaves[i].address < 16) Serial.print("0");
        Serial.println(slaves[i].address, HEX);

        // Process the received data based on command
        switch (command) {
          case 'A': {
            // Reconstruct floats
            float receivedBatVoltage, receivedBatCurrent, receivedSoC, receivedTemp, receivedVcc, receivedBalCurrent, receivedTerVoltage;
            uint8_t receivedFaults;
            memcpy(&receivedBatVoltage, &buffer[0], 4);
            memcpy(&receivedBatCurrent, &buffer[4], 4);
            memcpy(&receivedSoC, &buffer[8], 4);
            memcpy(&receivedTemp, &buffer[12], 4);
            memcpy(&receivedVcc, &buffer[16], 4);
            memcpy(&receivedBalCurrent, &buffer[20], 4);
            memcpy(&receivedTerVoltage, &buffer[24], 4);
            memcpy(&receivedFaults, &buffer[28], 1);

            Serial.print("Voltage: "); Serial.print(receivedBatVoltage, 2);
            Serial.print(" V, Current: "); Serial.print(receivedBatCurrent, 2);
            Serial.print(" A, SoC: "); Serial.print(receivedSoC, 1);
            Serial.println(" %");
            Serial.print("Vcc: "); Serial.print(receivedVcc, 2);
            Serial.println(" V");
            Serial.print("Temp: "); Serial.print(receivedTemp, 2);
            Serial.println(" C");   
            Serial.print("Balance Current: "); Serial.print(receivedBalCurrent, 2);
            Serial.println(" A"); 
            Serial.print("Terminal Voltage: "); Serial.print(receivedTerVoltage, 2);
            Serial.println(" V");
            Serial.print("Faults Reg: "); Serial.println(receivedFaults, BIN);
            Serial.println("----------------------------------------");
            break;
          }
          // Handle other global commands if needed
          default:
            Serial.println("Unhandled command.");
            break;
        }
      } else {
        Serial.print("Did not receive full data from slave 0x");
        if (slaves[i].address < 16) Serial.print("0");
        Serial.println(slaves[i].address, HEX);
      }
    } else {
      Serial.print("Failed to send command to slave 0x");
      if (slaves[i].address < 16) Serial.print("0");
      Serial.println(slaves[i].address, HEX);
    }
  }
}

void requestSlaveDataSingle(char command, uint8_t address) {
  // Request data from a single slave
  Wire.beginTransmission(address);
  Wire.write(command); // Send command (e.g., 'G', 'B', etc.)
  uint8_t error = Wire.endTransmission(false); // Send without releasing the bus

  if (error == 0) {
    // Determine expected data size based on command
    uint8_t expectedSize = 0;
    switch (command) {
      case 'G':
        expectedSize = (4 * 7) + 1; // Assuming 'G' is similar to 'A' for data retrieval
        break;
      case 'B':
        expectedSize = 1 + (4 * 3); // 13 bytes
        break;
      case 'P':
        expectedSize = 4 * 6; // 24 bytes
        break;
      // Add cases for other single-slave commands as needed
      default:
        expectedSize = 0;
        break;
    }

    if (expectedSize == 0) {
      Serial.println("Unknown command for single slave data request.");
      return;
    }

    Wire.requestFrom(address, expectedSize);
    byte buffer[32]; // Adjust size if needed
    uint8_t idx = 0;
    while (Wire.available() && idx < expectedSize) {
      buffer[idx++] = Wire.read();
    }

    if (idx == expectedSize) {
      // Update lastResponse timestamp
      for (uint8_t i = 0; i < slaveCount; i++) {
        if (slaves[i].address == address) {
          slaves[i].lastResponse = millis();
          break;
        }
      }

      Serial.print("Data from slave 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);

      // Process the received data based on command
      switch (command) {
        case 'G': {
          // Reconstruct floats (assuming same structure as 'A')
          float receivedBatVoltage, receivedBatCurrent, receivedSoC, receivedTemp, receivedVcc, receivedBalCurrent, receivedTerVoltage;
          uint8_t receivedFaults;
          memcpy(&receivedBatVoltage, &buffer[0], 4);
          memcpy(&receivedBatCurrent, &buffer[4], 4);
          memcpy(&receivedSoC, &buffer[8], 4);
          memcpy(&receivedTemp, &buffer[12], 4);
          memcpy(&receivedVcc, &buffer[16], 4);
          memcpy(&receivedBalCurrent, &buffer[20], 4);
          memcpy(&receivedTerVoltage, &buffer[24], 4);
          memcpy(&receivedFaults, &buffer[28], 1);

          Serial.print("Voltage: "); Serial.print(receivedBatVoltage, 2);
          Serial.print(" V, Current: "); Serial.print(receivedBatCurrent, 2);
          Serial.print(" A, SoC: "); Serial.print(receivedSoC, 1);
          Serial.println(" %");
          Serial.print("Vcc: "); Serial.print(receivedVcc, 2);
          Serial.println(" V");
          Serial.print("Temp: "); Serial.print(receivedTemp, 2);
          Serial.println(" C");   
          Serial.print("Balance Current: "); Serial.print(receivedBalCurrent, 2);
          Serial.println(" A"); 
          Serial.print("Terminal Voltage: "); Serial.print(receivedTerVoltage, 2);
          Serial.println(" V");
          Serial.print("Faults Reg: "); Serial.println(receivedFaults, BIN);
          Serial.println("----------------------------------------");
          break;
        }
        case 'B': {
          // Reconstruct floats
          uint8_t receivedBatteryType;
          float receivedFullVoltage, receivedEmptyVoltage, receivedBatteryCapacity;
          memcpy(&receivedBatteryType, &buffer[0], 1);
          memcpy(&receivedFullVoltage, &buffer[1], 4);
          memcpy(&receivedEmptyVoltage, &buffer[5], 4);
          memcpy(&receivedBatteryCapacity, &buffer[9], 4);

          Serial.print("Battery Type: ");
          if (receivedBatteryType == 1) Serial.println("LFP"); 
          else Serial.println("Li-ion");
          Serial.print("Full Voltage: "); Serial.print(receivedFullVoltage, 2);
          Serial.print(" V, Empty Voltage: "); Serial.print(receivedEmptyVoltage, 2);
          Serial.print(" V, Battery Capacity: "); Serial.print(receivedBatteryCapacity, 0);
          Serial.println(" mAh");
          Serial.println("----------------------------------------");
          break;
        }
        case 'P': {
          // Reconstruct floats
          float receivedI_chmax, receivedIdchmax, receivedOCVP, receivedODVP, receivedIBal, receivedOTemp;
          memcpy(&receivedI_chmax, &buffer[0], 4);
          memcpy(&receivedIdchmax, &buffer[4], 4);
          memcpy(&receivedOCVP, &buffer[8], 4);
          memcpy(&receivedODVP, &buffer[12], 4);
          memcpy(&receivedIBal, &buffer[16], 4);
          memcpy(&receivedOTemp, &buffer[20], 4);

          Serial.print("I_chmax: "); Serial.print(receivedI_chmax, 2);
          Serial.print(" A, I_dchmax: "); Serial.print(receivedIdchmax, 2);
          Serial.print(" A, OCVP: "); Serial.print(receivedOCVP, 2);
          Serial.print(" V, ODVP: "); Serial.print(receivedODVP, 2);
          Serial.print(" V, I Balance: "); Serial.print(receivedIBal, 2);
          Serial.print(" A, OTemp: "); Serial.print(receivedOTemp, 2);
          Serial.println(" C");
          Serial.println("----------------------------------------");
          break;
        }
        // Add cases for other single-slave commands as needed
        default:
          Serial.println("Unhandled command.");
          break;
      }
    } else {
      Serial.print("Did not receive full data from slave 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  } else {
    Serial.print("Failed to send command to slave 0x");
    if (address < 16) Serial.print("0");
    Serial.println(address, HEX);
  }
}

void setSlaveData(char command, uint8_t address, const char* params) {
  // Set data on a specific slave based on command and parameters
  Wire.beginTransmission(address);
  Wire.write(command);

  // Example: Set Battery Type ('C')
  if (command == 'C') {
    uint8_t batteryType = 1;          // 1 = LFP, 2 = Li-ion
    float batteryFullVoltage  = 3.65f; // For LFP, e.g., 3.65V
    float batteryEmptyVoltage = 2.50f; // Example
    float batteryCapacity     = 10.0f; // 10 Ah, for example

    Wire.write(batteryType);
    sendFloat(batteryFullVoltage);
    sendFloat(batteryEmptyVoltage);
    sendFloat(batteryCapacity);
  }

  // Handle other set commands ('Q', 'S', etc.)
  else if (command == 'Q') {
    float I_chmax = 5.00f;        // Max. Charging Current
    float I_dchmax = 8.00f;       // Max. Discharging Current
    float OCVP  = 3.65f;          // Over Charge Voltage Protection
    float ODVP = 2.00f;           // Over Discharge Voltage Protection
    float I_Bal = 0.00f;          // Max Balance Current
    float OTemp = 55.00f;         // Over Temp Threshold

    sendFloat(I_chmax);
    sendFloat(I_dchmax);
    sendFloat(OCVP);
    sendFloat(ODVP);
    sendFloat(I_Bal);
    sendFloat(OTemp);
  }

  else if (command == 'S') {
    // Send Switch Configuration (globally declared)
    Wire.write(SW_Config, sizeof(SW_Config));
  }

  Wire.endTransmission();
}

///////////////////////////////////////
// Helper Functions for Float Conversion
///////////////////////////////////////
void sendFloat(float value) {
  union {
    float f;
    uint8_t b[4];
  } data;
  data.f = value;
  Wire.write(data.b, 4);
}

float readFloat(const byte* buffer, int index) {
  union {
    float f;
    uint8_t b[4];
  } converter;

  for (int i = 0; i < 4; i++) {
    converter.b[i] = buffer[index + i];
  }
  return converter.f;
}

///////////////////////////////////////
// Serial Communication Handling
///////////////////////////////////////
void handleSerialInput() {
  while (Serial.available()) {
    char inChar = Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      serialBuffer[serialIndex] = '\0'; // Null-terminate the string
      serialIndex = 0;

      // Process the command
      if (strlen(serialBuffer) > 0) {
        // Tokenize the input based on space
        char* tokens[4]; // Command, Address, Param1, Param2
        uint8_t tokenCount = 0;
        char* ptr = strtok(serialBuffer, " ");

        while (ptr != NULL && tokenCount < 4) {
          tokens[tokenCount++] = ptr;
          ptr = strtok(NULL, " ");
        }

        if (tokenCount == 0) {
          Serial.println("No command entered.");
          return;
        }

        char command = tokens[0][0];
        uint8_t address = 0;

        switch (command) {
          case 'A':
            // 'A' command: Request data from all slaves
            Serial.println("Executing 'A' command: Collecting data from all slaves.");
            requestSlaveData('A');
            break;

          case 'G':
          case 'B':
          case 'C':
          case 'P':
          case 'Q':
          case 'S':
            // These commands require at least an address
            if (tokenCount < 2) {
              Serial.println("Error: Address parameter missing.");
              break;
            }

            // Parse the address (supports hexadecimal with '0x' prefix or decimal)
            if (strlen(tokens[1]) > 2 && tokens[1][0] == '0' && tokens[1][1] == 'x') {
              address = strtol(&tokens[1][2], NULL, 16);
            } else {
              address = strtol(tokens[1], NULL, 10);
            }

            // Validate the address
            bool validAddress = false;
            for (uint8_t i = 0; i < slaveCount; i++) {
              if (slaves[i].address == address) {
                validAddress = true;
                break;
              }
            }

            if (!validAddress) {
              Serial.print("Error: Slave address 0x");
              if (address < 16) Serial.print("0");
              Serial.print(address, HEX);
              Serial.println(" not found.");
              break;
            }

            // Handle the command
            switch (command) {
              case 'G':
                // 'G' command: Get data from a single slave
                Serial.print("Executing 'G' command: Collecting data from slave 0x");
                if (address < 16) Serial.print("0");
                Serial.println(address, HEX);
                requestSlaveDataSingle('G', address);
                break;

              case 'B':
              case 'P':
                // 'B' and 'P' commands: Get specific data from a single slave
                Serial.print("Executing '");
                Serial.print(command);
                Serial.print("' command: Collecting data from slave 0x");
                if (address < 16) Serial.print("0");
                Serial.println(address, HEX);
                requestSlaveDataSingle(command, address);
                break;

              case 'C':
              case 'Q':
              case 'S':
                // 'C', 'Q', 'S' commands: Set data on a single slave
                Serial.print("Executing '");
                Serial.print(command);
                Serial.print("' command: Setting data on slave 0x");
                if (address < 16) Serial.print("0");
                Serial.println(address, HEX);

                // Collect additional parameters if any
                const char* params = (tokenCount >= 3) ? tokens[2] : NULL;
                setSlaveData(command, address, params);
                break;

              default:
                Serial.println("Unknown Command.");
                break;
            }
            break;

          default:
            Serial.println("Unknown Command.");
            break;
        }
      }
    } else {
      if (serialIndex < (SERIAL_BUFFER_SIZE - 1)) {
        serialBuffer[serialIndex++] = inChar;
      }
    }
  }
  
}
