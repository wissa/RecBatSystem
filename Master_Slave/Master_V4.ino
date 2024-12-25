#include <Wire.h>

#define DEFAULT_SLAVE_ADDRESS 0x10
#define SLAVE_ADDRESS_START    0x11
#define SLAVE_ADDRESS_END      0x7F
#define COMMAND_SET_ADDRESS    0xA0

char sdatasize = 32;

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Master mode
  Wire.setClock(100000);
  delay(1000);
  Serial.println("Master ready.");
  assignAddresses();
}

void loop() {

   Serial.println("Send Command: ");    //A
    while (!Serial.available());
    char Userin = Serial.read();
    if (Userin == 'A' || Userin == 'B' || Userin == 'P' ) {
    sendCommand(Userin);
    Serial.println(Userin);
    RequestSlaveData (Userin, SLAVE_ADDRESS_START);
    }
    else if (Userin == 'C' || Userin == 'Q' || Userin == 'E' || Userin == 'S' ) {
    Serial.println(Userin);
    SetSlaveData (Userin, SLAVE_ADDRESS_START);
    }
  
  
  delay(500);
  
  }

///////////////////////////////////////
void assignAddresses() {
    uint8_t newAddress = SLAVE_ADDRESS_START;
    while (newAddress <= SLAVE_ADDRESS_END) {
        // Send address assignment command to DEFAULT_SLAVE_ADDRESS
        Wire.beginTransmission(DEFAULT_SLAVE_ADDRESS);
        Wire.write(COMMAND_SET_ADDRESS); // Command byte
        Wire.write(newAddress); // New address byte
        uint8_t error = Wire.endTransmission(true);

        if (error == 0) {
            Serial.print("Assigned new address 0x");
            if (newAddress < 16) Serial.print("0");
            Serial.println(newAddress, HEX);
            newAddress++;
            delay(100); // Short delay to allow slave to update
        } else {
            // No more slaves responding at default address
            Serial.println("No more slaves at default address.");
            break;
        }
    }
}

// Send a single-byte command to the slave
void sendCommand(char cmd) {
  Wire.beginTransmission(SLAVE_ADDRESS_START);
  Wire.write(cmd);             // e.g. 'V', 'I', 'S', 'A', 'L'
  Wire.endTransmission();      // Actually send the byte(s)
  delay(10);                   // Small delay to let Slave process
}

void RequestSlaveData (char command, char Slave_address){

// A: Get Online Data   , B: Get Battery Data, P: Get Protection Setting

if (command == 'A' ) {
  char datasize = (4*7)+1;   //8 Parameters, each 4 bytes as float
  Wire.requestFrom(Slave_address, datasize);
  byte buffer[datasize];
  int idx = 0;
  while (Wire.available() && idx < sdatasize) {buffer[idx++] = Wire.read();}
  if (idx == datasize) {
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
  } else {
    Serial.println("Did not receive full data from slave!");
  }
  }

else if (command == 'B') {
  char datasize = 1+(4*3);
  Wire.requestFrom(Slave_address, datasize);
  byte buffer[datasize];
  int idx = 0;
  while (Wire.available() && idx < datasize) {buffer[idx++] = Wire.read();}
  if (idx == datasize) {
    // Reconstruct floats
    uint8_t receivedbatteryType;
    float receivedfullVoltage, receivedemptyVoltage, receivedbatteryCapacity;
    memcpy(&receivedbatteryType, &buffer[0], 1);
    memcpy(&receivedfullVoltage, &buffer[1], 4);
    memcpy(&receivedemptyVoltage, &buffer[5], 4);
    memcpy(&receivedbatteryCapacity, &buffer[9], 4);

    Serial.print("Battery Type: ");
    if (receivedbatteryType==1) Serial.println("LFP"); else Serial.println ("Li-ion");
    Serial.print("Full Voltage: "); Serial.print(receivedfullVoltage, 2);
    Serial.print(" V, Empty Voltage: "); Serial.print(receivedemptyVoltage, 2);
    Serial.print(" V, Battery Capacity: "); Serial.print(receivedbatteryCapacity, 0);
    Serial.println(" mAh");
  }
}

////////////// Get Protection Setting ///////////////
else if (command == 'P') {
  char datasize = (4*6);
  Wire.requestFrom(Slave_address, datasize);
  byte buffer[datasize];
  int idx = 0;
  while (Wire.available() && idx < datasize) {buffer[idx++] = Wire.read();}
  if (idx == datasize) {
    // Reconstruct floats
    uint8_t receivedbatteryType;
    float receivedI_chmax, receivedemptyI_dchmax, receivedOCVP, receivedODVP , receivedI_Bal , receivedOTemp;
    memcpy(&receivedI_chmax, &buffer[0], 4);
    memcpy(&receivedemptyI_dchmax, &buffer[4], 4);
    memcpy(&receivedOCVP, &buffer[8], 4);
    memcpy(&receivedODVP, &buffer[12], 4);
    memcpy(&receivedI_Bal, &buffer[16], 4);
    memcpy(&receivedOTemp, &buffer[20], 4);

    Serial.print("Ich max: "); Serial.print(receivedI_chmax, 2);
    Serial.print(" A, Idch max: "); Serial.print(receivedemptyI_dchmax, 2);
    Serial.print(" A, OCVP: "); Serial.print(receivedOCVP, 2);
    Serial.print(" V, ODVP: "); Serial.print(receivedODVP, 2);
    Serial.print(" V, I Balance: "); Serial.print(receivedI_Bal, 2);
    Serial.print(" A, OTemp: "); Serial.print(receivedOTemp, 2);
    Serial.println(" C");
  }
}


}


void SetSlaveData (char command, char Slave_address){
//C: Set battery setting, Q: Set Protection settings, E: Set max balance current, S: Set SW status 

if (command == 'C' ) {
  uint8_t batteryType = 1;       // 1 = LFP, 2 = Li-ion
  float batteryFullVoltage  = 3.65f; // For LFP, e.g. 3.65V
  float batteryEmptyVoltage = 2.50f; // Example
  float batteryCapacity     = 10.0f; // 10 Ah, for example
Wire.beginTransmission(Slave_address);
Wire.write('C');
Wire.write(batteryType);
sendFloat(batteryFullVoltage);
sendFloat(batteryEmptyVoltage);
sendFloat(batteryCapacity);
Wire.endTransmission();
command = 0;
}
////////////// Set Protection setting ////////////////
else if (command == 'Q' ) {
  float I_chmax = 5.00f;       // Max. Charging Current
  float I_dchmax = 8.00f;       // Max. DisCharging Current
  float OCVP  = 3.65f; // Over Charge voltage protection
  float ODVP = 2.00f; // Over DisCharge voltage protection
  float I_Bal = 0.00f; // Max Balance current
  float OTemp = 55.00f; // Over Temp Threshold
Wire.beginTransmission(Slave_address);
Wire.write('Q');
sendFloat(I_chmax);
sendFloat(I_dchmax);
sendFloat(OCVP);
sendFloat(ODVP);
sendFloat(I_Bal);
sendFloat(OTemp);
Wire.endTransmission();
command = 0;
}

////////////// Set Swtiches setting ////////////////
else if (command == 'S' ) {
  uint8_t SW_Config = 0b10101011;       // Max. Charging Current
Wire.beginTransmission(Slave_address);
Wire.write('S');
Wire.write(SW_Config);
Wire.endTransmission();
command = 0;
}
}

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
