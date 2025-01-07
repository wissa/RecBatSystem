#include <Wire.h>
#include <EEPROM.h>

/*
  Redesigned Master Code
  ------------------------------------------------------------
  Key Additions to address your requests:
  1) Prevent repeated removal/addition of the same slave by using
     a `consecutiveTimeouts` counter in `Slave`.
  2) New commands to set Battery/Protection/Intervals from Serial:
     - SETBATT <batteryType> <fullV> <emptyV> <capacity>
     - SETPROT <I_chmax> <I_dchmax> <OCVP> <ODVP> <I_Bal> <OverTemp>
     - SETINT <addrAssignMS> <dataCollectMS> <slaveTimeoutMS>
  3) Save/Load from EEPROM still works via `SAVE` and `LOAD`.
  4) The existing commands (A, B, C, P, Q, S, R, F, etc.) remain
     and function as before.
*/

// --------------------------- I2C & Slave Settings -------------------------- //
#define DEFAULT_SLAVE_ADDRESS    0x10
#define SLAVE_ADDRESS_START      0x11
#define SLAVE_ADDRESS_END        0x7F
#define COMMAND_SET_ADDRESS      0xA0

// --------------------------- Timing Intervals ------------------------------ //
// Default intervals (can be changed at runtime via SETINT)
unsigned long addressAssignInterval  = 10000UL; // 10s
unsigned long dataCollectionInterval = 2000UL;  // 2s
unsigned long slaveTimeout           = 15000UL; // 15s

// Sync Pin & Timer for 20 kHz square wave
#define SYNC_PIN 2  // PD2 on AVR
// We'll update the frequency by command "F <freq>"

// Maximum number of slaves
#define MAX_SLAVES 50

// Serial command buffer
#define SERIAL_BUFFER_SIZE 64
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialIndex = 0;

// Data printing control
bool printEnabled = true;
uint8_t buffer[29];

// --------------------- EEPROM Addresses for Master Data -------------------- //
/*
   We'll store in EEPROM:
   - Battery: type(1B), fullV(4B), emptyV(4B), capacity(4B)  => 13 bytes
   - Protection: I_chmax(4B), I_dchmax(4B), OCVP(4B),
                 ODVP(4B), I_Bal(4B), OverTemp(4B) => 24 bytes
   - Switch Patterns: 16 bytes
   - Intervals: 3 x 4 bytes => 12 bytes
*/
#define MASTER_BATT_TYPE_ADDR        0
#define MASTER_BATT_FULLV_ADDR       1
#define MASTER_BATT_EMPTYV_ADDR      5
#define MASTER_BATT_CAPACITY_ADDR    9
#define MASTER_PROT_ICHMAX_ADDR      13
#define MASTER_PROT_IDCHMAX_ADDR     17
#define MASTER_PROT_OCVP_ADDR        21
#define MASTER_PROT_ODVP_ADDR        25
#define MASTER_PROT_IBAL_ADDR        29
#define MASTER_PROT_OVERTEMP_ADDR    33
#define MASTER_SW_PATTERNS_ADDR      37  // next 16 bytes => 37..52

// We'll store intervals at 53..(53+12)
#define MASTER_ADDR_ASSIGN_INTERVAL_ADDR   53  // 4 bytes
#define MASTER_DATA_COLLECTION_INTERVAL_ADDR 57 // 4 bytes
#define MASTER_SLAVE_TIMEOUT_ADDR          61  // 4 bytes

// --------------------- Structures / Global Vars ---------------------------- //
struct Slave {
  uint8_t address;
  unsigned long lastResponse;
  uint8_t  consecutiveTimeouts; // new field to help stable presence
};

Slave slaves[MAX_SLAVES];
uint8_t slaveCount = 0;
uint8_t nextAddress = SLAVE_ADDRESS_START;

// Master’s default Battery config (from EEPROM)
uint8_t masterBatteryType;  
float   masterFullVoltage;
float   masterEmptyVoltage;
float   masterCapacity;

// Master’s default Protection config
float masterI_chmax;
float masterI_dchmax;
float masterOCVP;
float masterODVP;
float masterI_Bal;
float masterOverTemp;

// Switch patterns
#define SW_PATTERN_LENGTH 16
uint8_t masterSW_Config[SW_PATTERN_LENGTH];

// Non-blocking timing
unsigned long previousAddressAssign  = 0;
unsigned long previousDataCollection = 0;

// --------------------------- Frequency Helpers ------------------------------ //
/*
   By default ~20 kHz on Timer1, toggling PD2 (SYNC_PIN).
   Formula for an AVR @16MHz, prescaler=8:

   freq = 1e6 / (OCR1A+1)

   We'll allow user to do "F <freq>" to recalc OCR1A.
*/
uint16_t calcOCR1AforFreq(uint32_t freq) {
  if (freq < 1) freq = 1;
  uint32_t tmp = (1000000UL / freq);
  if (tmp == 0) tmp = 1;
  return (uint16_t)(tmp - 1);
}

// -------------------------- Forward Declarations --------------------------- //
void loadMasterSettingsFromEEPROM();
void saveMasterSettingsToEEPROM();
void initTimer1(uint32_t freq);

void assignAddresses();
void checkForNewSlaves();
bool isAddressInUse(uint8_t address);
void removeSlave(uint8_t index);

void requestSlaveDataAll();
void Print_SlavesData();
void requestSlaveDataSingle(char command, uint8_t address);
void setSlaveData(char command, uint8_t address);

void sendFloat(float value);
float readFloat(const byte* buffer, int index);

void handleSerialInput();
void parseCommand(char* cmdLine);

// Timer1 Compare Match ISR: toggle SYNC_PIN
ISR(TIMER1_COMPA_vect) {
  // Toggle PD2
  PIND = (1 << PIND2); 
}

// ------------------------------ setup() ------------------------------------ //
void setup() {
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  loadMasterSettingsFromEEPROM();
  // Default freq = 20kHz
  initTimer1(20000);

  Serial.println(F("Master Ready."));

  // Initial address assignment
  assignAddresses();
}

// ------------------------------ loop() ------------------------------------- //
void loop() {
  unsigned long now = millis();

  // Handle serial input
  handleSerialInput();

  // Periodic address assignment
  if (now - previousAddressAssign >= addressAssignInterval) {
    previousAddressAssign = now;
    assignAddresses();
    checkForNewSlaves();
  }

  // Periodic data collection
  if (now - previousDataCollection >= dataCollectionInterval) {
    previousDataCollection = now;
      requestSlaveDataAll(); // 'A'
      
    
  }

  // Check timeouts
  for (uint8_t i = 0; i < slaveCount; i++) {
    unsigned long elapsed = now - slaves[i].lastResponse;
    if (elapsed > slaveTimeout) {
      // Increment consecutive timeouts
      slaves[i].consecutiveTimeouts++;
      if (slaves[i].consecutiveTimeouts >= 3) {
        Serial.print(F("Slave 0x"));
        if (slaves[i].address < 16) Serial.print('0');
        Serial.print(slaves[i].address, HEX);
        Serial.println(F(" timed out multiple times. Removing."));
        removeSlave(i);
        i--; 
      }
    }
  }
}

// --------------------- EEPROM Load/Save Master Data ------------------------ //
void loadMasterSettingsFromEEPROM() {
  // Battery
  masterBatteryType = EEPROM.read(MASTER_BATT_TYPE_ADDR);
  EEPROM.get(MASTER_BATT_FULLV_ADDR,   masterFullVoltage);
  EEPROM.get(MASTER_BATT_EMPTYV_ADDR,  masterEmptyVoltage);
  EEPROM.get(MASTER_BATT_CAPACITY_ADDR,masterCapacity);

  // If invalid, set defaults
  if (masterBatteryType == 0xFF || masterBatteryType == 0) {
    masterBatteryType = 1; // LFP
    masterFullVoltage = 3.60f;
    masterEmptyVoltage= 2.50f;
    masterCapacity    = 10.0f;
  }

  // Protection
  EEPROM.get(MASTER_PROT_ICHMAX_ADDR,   masterI_chmax);
  EEPROM.get(MASTER_PROT_IDCHMAX_ADDR,  masterI_dchmax);
  EEPROM.get(MASTER_PROT_OCVP_ADDR,     masterOCVP);
  EEPROM.get(MASTER_PROT_ODVP_ADDR,     masterODVP);
  EEPROM.get(MASTER_PROT_IBAL_ADDR,     masterI_Bal);
  EEPROM.get(MASTER_PROT_OVERTEMP_ADDR, masterOverTemp);

  if (isnan(masterI_chmax)) {
    masterI_chmax  = 5.0f;
    masterI_dchmax = 8.0f;
    masterOCVP     = 3.65f;
    masterODVP     = 2.0f;
    masterI_Bal    = 0.0f;
    masterOverTemp = 55.0f;
  }

  // Switch patterns
  for (int i = 0; i < SW_PATTERN_LENGTH; i++) {
    masterSW_Config[i] = EEPROM.read(MASTER_SW_PATTERNS_ADDR + i);
    if (masterSW_Config[i] == 0xFF) {
      masterSW_Config[i] = (i % 2) ? 0x55 : 0xAA;
    }
  }

  // Intervals
  EEPROM.get(MASTER_ADDR_ASSIGN_INTERVAL_ADDR,   addressAssignInterval);
  EEPROM.get(MASTER_DATA_COLLECTION_INTERVAL_ADDR,dataCollectionInterval);
  EEPROM.get(MASTER_SLAVE_TIMEOUT_ADDR,          slaveTimeout);

  // Validate
  if (addressAssignInterval == 0xFFFFFFFF) addressAssignInterval  = 10000UL;
  if (dataCollectionInterval == 0xFFFFFFFF) dataCollectionInterval= 2000UL;
  if (slaveTimeout == 0xFFFFFFFF)          slaveTimeout           = 15000UL;
}

void saveMasterSettingsToEEPROM() {
  // Battery
  EEPROM.write(MASTER_BATT_TYPE_ADDR, masterBatteryType);
  EEPROM.put(MASTER_BATT_FULLV_ADDR,   masterFullVoltage);
  EEPROM.put(MASTER_BATT_EMPTYV_ADDR,  masterEmptyVoltage);
  EEPROM.put(MASTER_BATT_CAPACITY_ADDR,masterCapacity);

  // Protection
  EEPROM.put(MASTER_PROT_ICHMAX_ADDR,   masterI_chmax);
  EEPROM.put(MASTER_PROT_IDCHMAX_ADDR,  masterI_dchmax);
  EEPROM.put(MASTER_PROT_OCVP_ADDR,     masterOCVP);
  EEPROM.put(MASTER_PROT_ODVP_ADDR,     masterODVP);
  EEPROM.put(MASTER_PROT_IBAL_ADDR,     masterI_Bal);
  EEPROM.put(MASTER_PROT_OVERTEMP_ADDR, masterOverTemp);

  // Switch patterns
  for (int i = 0; i < SW_PATTERN_LENGTH; i++) {
    EEPROM.write(MASTER_SW_PATTERNS_ADDR + i, masterSW_Config[i]);
  }

  // Intervals
  EEPROM.put(MASTER_ADDR_ASSIGN_INTERVAL_ADDR,   addressAssignInterval);
  EEPROM.put(MASTER_DATA_COLLECTION_INTERVAL_ADDR,dataCollectionInterval);
  EEPROM.put(MASTER_SLAVE_TIMEOUT_ADDR,          slaveTimeout);
}

// ----------------------- Timer1 Initialization ----------------------------- //
void initTimer1(uint32_t freq) {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // CTC mode
  TCCR1B |= (1 << WGM12);
  // Prescaler = 8
  TCCR1B |= (1 << CS11);

  uint16_t val = calcOCR1AforFreq(freq);
  OCR1A = val;

  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

// ------------------ Assign & Detect Slaves (Stabilized) -------------------- //
void assignAddresses() {
  if (nextAddress > SLAVE_ADDRESS_END || slaveCount >= MAX_SLAVES) return;

  Wire.beginTransmission(DEFAULT_SLAVE_ADDRESS);
  Wire.write(COMMAND_SET_ADDRESS);
  Wire.write(nextAddress);
  uint8_t err = Wire.endTransmission(true);

  if (err == 0) {
    // Found a new device at default address
    slaves[slaveCount].address = nextAddress;
    slaves[slaveCount].lastResponse = millis();
    slaves[slaveCount].consecutiveTimeouts = 0;
    slaveCount++;
    Serial.print(F("New Slave assigned: 0x"));
    if (nextAddress < 16) Serial.print('0');
    Serial.println(nextAddress, HEX);

    nextAddress++;
  } else {
    nextAddress = SLAVE_ADDRESS_START; // reset
  }
}

void checkForNewSlaves() {
  bool foundAny = false;
  for (uint8_t addr = SLAVE_ADDRESS_START; addr <= SLAVE_ADDRESS_END; addr++) {
    if (!isAddressInUse(addr)) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0) {
        // Found a new slave
        if (slaveCount < MAX_SLAVES) {
          slaves[slaveCount].address = addr;
          slaves[slaveCount].lastResponse = millis();
          slaves[slaveCount].consecutiveTimeouts = 0;
          slaveCount++;
          foundAny = true;
          Serial.print(F("Detected new slave at 0x"));
          if (addr < 16) Serial.print('0');
          Serial.println(addr, HEX);
        }
      }
    }
  }
  // We only print if we found something new
}

bool isAddressInUse(uint8_t address) {
  for (uint8_t i = 0; i < slaveCount; i++) {
    if (slaves[i].address == address) return true;
  }
  return false;
}

void removeSlave(uint8_t index) {
  if (index >= slaveCount) return;
  for (uint8_t i = index; i < slaveCount - 1; i++) {
    slaves[i] = slaves[i + 1];
  }
  slaveCount--;
}

// ------------------ Request Data from Slaves (All / Single) ---------------- //
void requestSlaveDataAll() {
  // Command 'A' => 29 bytes from each slave
  for (uint8_t i = 0; i < slaveCount; i++) {
    Wire.beginTransmission(slaves[i].address);
    Wire.write('A');
    uint8_t error = Wire.endTransmission(false);
    if (error == 0) {
      // Expect 29 bytes
      uint8_t expectedSize = 29;
      Wire.requestFrom(slaves[i].address, expectedSize);
      
      uint8_t idx = 0;
      while (Wire.available() && idx < expectedSize) {
        buffer[idx++] = Wire.read();
      }
      if (idx == expectedSize) {
        // Mark lastResponse, reset consecutive timeouts
        slaves[i].lastResponse = millis();
        slaves[i].consecutiveTimeouts = 0;

        if (printEnabled) {
            Print_SlavesData(i);
        }
      }
    }
  }
}

void Print_SlavesData(uint8_t i)
{
        if (i != 0) {
        float batVoltage, batCurrent, soc, temp, vcc, balCurrent, terVoltage;
        uint8_t faults;
        memcpy(&batVoltage,   &buffer[0],  4);
        memcpy(&batCurrent,   &buffer[4],  4);
        memcpy(&soc,          &buffer[8],  4);
        memcpy(&temp,         &buffer[12], 4);
        memcpy(&vcc,          &buffer[16], 4);
        memcpy(&balCurrent,   &buffer[20], 4);
        memcpy(&terVoltage,   &buffer[24], 4);
        memcpy(&faults,       &buffer[28], 1);

        Serial.print(F("Slave 0x"));
        if (slaves[i].address < 16) Serial.print('0');
        Serial.print(slaves[i].address, HEX);
        Serial.print(F(" => V="));   Serial.print(batVoltage, 4);
        Serial.print(F("V, I="));    Serial.print(batCurrent, 4);
        Serial.print(F("A, SoC="));  Serial.print(soc, 2);
        Serial.print(F("%, T="));    Serial.print(temp, 2);
        Serial.print(F("C, Vcc="));  Serial.print(vcc, 2);
        Serial.print(F("V, BalI=")); Serial.print(balCurrent, 4);
        Serial.print(F("A, TerV=")); Serial.print(terVoltage, 4);
        Serial.print(F("V, FaultReg="));
        Serial.println(faults, BIN);
        }
        else
        {
          for (uint8_t i = 0; i < slaveCount; i++) {
            float batVoltage, batCurrent, soc, temp, vcc, balCurrent, terVoltage;
        uint8_t faults;
        memcpy(&batVoltage,   &buffer[0],  4);
        memcpy(&batCurrent,   &buffer[4],  4);
        memcpy(&soc,          &buffer[8],  4);
        memcpy(&temp,         &buffer[12], 4);
        memcpy(&vcc,          &buffer[16], 4);
        memcpy(&balCurrent,   &buffer[20], 4);
        memcpy(&terVoltage,   &buffer[24], 4);
        memcpy(&faults,       &buffer[28], 1);

        Serial.print(F("Slave 0x"));
        if (slaves[i].address < 16) Serial.print('0');
        Serial.print(slaves[i].address, HEX);
        Serial.print(F(" => V="));   Serial.print(batVoltage, 4);
        Serial.print(F("V, I="));    Serial.print(batCurrent, 4);
        Serial.print(F("A, SoC="));  Serial.print(soc, 2);
        Serial.print(F("%, T="));    Serial.print(temp, 2);
        Serial.print(F("C, Vcc="));  Serial.print(vcc, 2);
        Serial.print(F("V, BalI=")); Serial.print(balCurrent, 4);
        Serial.print(F("A, TerV=")); Serial.print(terVoltage, 4);
        Serial.print(F("V, FaultReg="));
        Serial.println(faults, BIN);
          }
        }
}

void requestSlaveDataSingle(char command, uint8_t address) {
  // e.g. 'B' => battery (13 bytes), 'P' => protection (24 bytes)
  Wire.beginTransmission(address);
  Wire.write(command);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Serial.print(F("No response from 0x"));
    if (address < 16) Serial.print('0');
    Serial.println(address, HEX);
    return;
  }

  uint8_t expectedSize = 0;
  if (command == 'B') expectedSize = 13; 
  else if (command == 'P') expectedSize = 24; 
  else return; 

  Wire.requestFrom(address, expectedSize);
  uint8_t buffer[24];
  uint8_t idx = 0;
  while (Wire.available() && idx < expectedSize) {
    buffer[idx++] = Wire.read();
  }

  if (idx == expectedSize) {
    // Reset timeout
    for (uint8_t i = 0; i < slaveCount; i++) {
      if (slaves[i].address == address) {
        slaves[i].lastResponse = millis();
        slaves[i].consecutiveTimeouts = 0;
        break;
      }
    }
    // Parse
    if (command == 'B') {
      uint8_t batType;
      float fV,eV,cap;
      memcpy(&batType, &buffer[0],1);
      memcpy(&fV,      &buffer[1],4);
      memcpy(&eV,      &buffer[5],4);
      memcpy(&cap,     &buffer[9],4);
      Serial.print(F("Slave 0x"));
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
      Serial.print(F(" => BatteryType="));
      Serial.print(batType);
      Serial.print(F(", FullV=")); Serial.print(fV,2);
      Serial.print(F(", EmptyV="));Serial.print(eV,2);
      Serial.print(F(", Capacity="));Serial.println(cap,2);
    }
    else if (command == 'P') {
      float I_ch,I_dch,ocvp,odvp,iBal,tMax;
      memcpy(&I_ch,   &buffer[0], 4);
      memcpy(&I_dch,  &buffer[4], 4);
      memcpy(&ocvp,   &buffer[8], 4);
      memcpy(&odvp,   &buffer[12],4);
      memcpy(&iBal,   &buffer[16],4);
      memcpy(&tMax,   &buffer[20],4);
      Serial.print(F("Slave 0x"));
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
      Serial.print(F(" => I_ch="));   Serial.print(I_ch,2);
      Serial.print(F("A, I_dch="));   Serial.print(I_dch,2);
      Serial.print(F("A, OCVP="));    Serial.print(ocvp,2);
      Serial.print(F("V, ODVP="));    Serial.print(odvp,2);
      Serial.print(F("V, I_Bal="));   Serial.print(iBal,4);
      Serial.print(F("A, OverT="));   Serial.println(tMax,2);
    }
  }
  else {
    Serial.print(F("Partial data from 0x"));
    if (address < 16) Serial.print('0');
    Serial.println(address, HEX);
  }
}

// ------------------ Set Data on Slave (C, Q, S, etc.) ---------------------- //
void setSlaveData(char command, uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(command);

  if (command == 'C') {
    // Send Master’s Battery config
    Wire.write(masterBatteryType);
    sendFloat(masterFullVoltage);
    sendFloat(masterEmptyVoltage);
    sendFloat(masterCapacity);
  }
  else if (command == 'Q') {
    // Send Master’s Protection config
    sendFloat(masterI_chmax);
    sendFloat(masterI_dchmax);
    sendFloat(masterOCVP);
    sendFloat(masterODVP);
    sendFloat(masterI_Bal);
    sendFloat(masterOverTemp);
  }
  else if (command == 'S') {
    // Send Master’s switch patterns
    Wire.write(masterSW_Config, SW_PATTERN_LENGTH);
  }

  Wire.endTransmission();
}

// ------------------- Float Helpers (unchanged) ----------------------------- //
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

// ----------------------- Serial Input Parsing ------------------------------ //
void handleSerialInput() {
  while (Serial.available()) {
    char inChar = Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        parseCommand(serialBuffer);
        serialIndex = 0;
      }
    } else {
      if (serialIndex < (SERIAL_BUFFER_SIZE - 1)) {
        serialBuffer[serialIndex++] = inChar;
      }
    }
  }
}

void parseCommand(char* cmdLine) {
  char* token = strtok(cmdLine, " ");
  if (!token) return;

  // Examples:
  // A => request data from all slaves (prints if enabled)
  // PRINT ON / OFF
  // B 0x11 => get Battery from slave 0x11
  // C 0x11 => set Battery on slave 0x11
  // C ALL  => set Battery on all slaves
  // R ...  => set Master’s switch patterns
  // S 0x11 => send patterns to slave 0x11
  // F 20000 => set freq to 20kHz
  // SAVE / LOAD
  // SETBATT <batType> <fullV> <emptyV> <cap>
  // SETPROT <I_ch> <I_dch> <OCVP> <ODVP> <I_Bal> <OverT>
  // SETINT <addrInterval> <dataInterval> <slaveTimeout>

  if (!strcmp(token, "A")) {
    // Force data collection from all
    requestSlaveDataAll();
    Print_SlavesData(0);
    //if (printEnabled) requestSlaveDataAll();
    //else Serial.println(F("Printing disabled. (Data polled silently if needed.)"));
  }
  else if (!strcmp(token, "PRINT")) {
    char* arg = strtok(NULL, " ");
    if (!arg) return;
    if (!strcmp(arg, "ON")) {
      printEnabled = true;
      Serial.println(F("Data printing ENABLED."));
    } else if (!strcmp(arg, "OFF")) {
      printEnabled = false;
      Serial.println(F("Data printing DISABLED."));
    }
  }
  else if (!strcmp(token, "B") || !strcmp(token, "P")) {
    // e.g. B 0x11 or P 0x11 or B ALL
    char command = token[0];
    char* addrStr = strtok(NULL, " ");
    if (!addrStr) return;

    bool doAll = false;
    uint8_t address = 0;
    if (!strcmp(addrStr, "ALL")) {
      doAll = true;
    } else {
      // parse hex or decimal
      if (strlen(addrStr) > 2 && (addrStr[0] == '0') && (addrStr[1] == 'x')) {
        address = strtol(&addrStr[2], NULL, 16);
      } else {
        address = strtol(addrStr, NULL, 10);
      }
    }

    if (doAll) {
      for (uint8_t i = 0; i < slaveCount; i++) {
        requestSlaveDataSingle(command, slaves[i].address);
      }
    } else {
      bool found = false;
      for (uint8_t i = 0; i < slaveCount; i++) {
        if (slaves[i].address == address) {
          requestSlaveDataSingle(command, address);
          found = true;
          break;
        }
      }
      if (!found) {
        Serial.print(F("Slave 0x"));
        if (address < 16) Serial.print('0');
        Serial.print(address, HEX);
        Serial.println(F(" not found."));
      }
    }
  }
  else if (!strcmp(token, "C") || !strcmp(token, "Q") || !strcmp(token, "S")) {
    // e.g. C 0x11, Q 0x11, S 0x11
    char command = token[0];
    char* addrStr = strtok(NULL, " ");
    if (!addrStr) return;

    bool doAll = false;
    uint8_t address = 0;
    if (!strcmp(addrStr, "ALL")) {
      doAll = true;
    } else {
      if (strlen(addrStr) > 2 && addrStr[0] == '0' && addrStr[1] == 'x') {
        address = strtol(&addrStr[2], NULL, 16);
      } else {
        address = strtol(addrStr, NULL, 10);
      }
    }

    if (doAll) {
      for (uint8_t i = 0; i < slaveCount; i++) {
        setSlaveData(command, slaves[i].address);
      }
      Serial.print(F("Command '"));
      Serial.print(command);
      Serial.println(F("' sent to ALL slaves."));
    } else {
      bool found = false;
      for (uint8_t i = 0; i < slaveCount; i++) {
        if (slaves[i].address == address) {
          setSlaveData(command, address);
          found = true;
          Serial.print(F("Command '"));
          Serial.print(command);
          Serial.print(F("' sent to slave 0x"));
          if (address < 16) Serial.print('0');
          Serial.println(address, HEX);
          break;
        }
      }
      if (!found) {
        Serial.print(F("Slave 0x"));
        if (address < 16) Serial.print('0');
        Serial.print(address, HEX);
        Serial.println(F(" not found."));
      }
    }
  }
  else if (!strcmp(token, "R")) {
    // R => read up to 16 values to update masterSW_Config
    for (int i = 0; i < SW_PATTERN_LENGTH; i++) {
      char* p = strtok(NULL, " ");
      if (!p) break;
      uint8_t val = 0;
      if (strlen(p) > 2 && p[0] == '0' && p[1] == 'x') {
        val = strtol(&p[2], NULL, 16);
      } else {
        val = (uint8_t)strtol(p, NULL, 10);
      }
      masterSW_Config[i] = val;
    }
    Serial.println(F("Master switch patterns updated (RAM). Use SAVE to store in EEPROM."));
  }
  else if (!strcmp(token, "F")) {
    // set SYNC freq
    char* arg = strtok(NULL, " ");
    if (!arg) return;
    uint32_t newFreq = strtoul(arg, NULL, 10);
    initTimer1(newFreq);
    Serial.print(F("SYNC freq set to "));
    Serial.print(newFreq);
    Serial.println(F(" Hz."));
  }
  else if (!strcmp(token, "SAVE")) {
    saveMasterSettingsToEEPROM();
    Serial.println(F("Master settings saved to EEPROM."));
  }
  else if (!strcmp(token, "LOAD")) {
    loadMasterSettingsFromEEPROM();
    Serial.println(F("Master settings reloaded from EEPROM."));
  }
  else if (!strcmp(token, "SETBATT")) {
    // e.g. SETBATT 2 4.20 3.00 5.0
    // parse
    char* p1 = strtok(NULL, " "); // batteryType
    char* p2 = strtok(NULL, " "); // fullV
    char* p3 = strtok(NULL, " "); // emptyV
    char* p4 = strtok(NULL, " "); // capacity
    if (!p1 || !p2 || !p3 || !p4) {
      Serial.println(F("Usage: SETBATT <type> <fullV> <emptyV> <capacity>"));
      return;
    }
    masterBatteryType  = (uint8_t)strtol(p1, NULL, 10);
masterFullVoltage  = atof(p2);
masterEmptyVoltage = atof(p3);
masterCapacity     = atof(p4);

    Serial.println(F("Master Battery settings updated (RAM). Use SAVE to store in EEPROM."));
  }
  else if (!strcmp(token, "SETPROT")) {
    // e.g. SETPROT 5.0 8.0 3.65 2.00 0.0 55.0
    // parse 6 floats
    char* p1 = strtok(NULL, " "); // I_chmax
    char* p2 = strtok(NULL, " "); // I_dchmax
    char* p3 = strtok(NULL, " "); // OCVP
    char* p4 = strtok(NULL, " "); // ODVP
    char* p5 = strtok(NULL, " "); // I_Bal
    char* p6 = strtok(NULL, " "); // OverTemp
    if (!p1 || !p2 || !p3 || !p4 || !p5 || !p6) {
      Serial.println(F("Usage: SETPROT <I_chmax> <I_dchmax> <OCVP> <ODVP> <I_Bal> <OverTemp>"));
      return;
    }
 masterI_chmax  = atof(p1);
masterI_dchmax = atof(p2);
masterOCVP     = atof(p3);
masterODVP     = atof(p4);
masterI_Bal    = atof(p5);
masterOverTemp = atof(p6);

    Serial.println(F("Master Protection settings updated (RAM). Use SAVE to store in EEPROM."));
  }
  else if (!strcmp(token, "SETINT")) {
    // e.g. SETINT 10000 2000 15000
    // parse 3 unsigned longs
    char* p1 = strtok(NULL, " ");
    char* p2 = strtok(NULL, " ");
    char* p3 = strtok(NULL, " ");
    if (!p1 || !p2 || !p3) {
      Serial.println(F("Usage: SETINT <addrAssign_ms> <dataCollect_ms> <slaveTimeout_ms>"));
      return;
    }
    addressAssignInterval  = strtoul(p1, NULL, 10);
    dataCollectionInterval = strtoul(p2, NULL, 10);
    slaveTimeout           = strtoul(p3, NULL, 10);
    Serial.println(F("Master intervals updated (RAM). Use SAVE to store in EEPROM."));
  }
  else {
    Serial.println(F("Unknown command."));
  }
}
