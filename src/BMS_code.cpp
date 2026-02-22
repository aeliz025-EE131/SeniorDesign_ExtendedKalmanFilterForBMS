#include <Arduino.h>
#include <Wire.h>

#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo-1)*2))
#define packVolt 0x36;


unsigned int buttonA_pin =4 ;
unsigned int buttonB_pin =5 ;
unsigned int redLED = 3;
unsigned int greenLED =2;
unsigned int num_cell = 10;



const uint8_t BQ_ADDR = 0x08;   // <-- set to your BQ76942 I2C address

int16_t directCommand(byte command) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(command);
  Wire.endTransmission();

  Wire.requestFrom(BQ_ADDR, 2);
  while(!Wire.available());
  byte lsb = Wire.read();
  byte msb = Wire.read();

  return (unsigned int)(msb << 8 | lsb);
}

void sendSubcommand(uint16_t cmd)
{
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(0x3E);
  Wire.write(cmd & 0xFF);   // LSB
  Wire.write(cmd >> 8);     // MSB
  Wire.endTransmission();
  delay(2);
}


// Helper: write a single-byte command register (0x00–0x7F)
void bqWriteCmd(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Helper: read 2 bytes from a command register (e.g. 0x12 BatteryStatus)
uint16_t bqReadWord(uint8_t reg) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ_ADDR, (uint8_t)2);
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

uint16_t bqReadDataMemWord(uint16_t addr)
{
  uint8_t subLo = addr & 0xFF;
  uint8_t subHi = (addr >> 8) & 0xFF;

  // 1) Write subcommand address to 0x3E/0x3F
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(0x3E);      // subcommand low-byte register
  Wire.write(subLo);
  Wire.write(subHi);
  Wire.endTransmission();

  // 2) Poll 0x3E/0x3F until the device has completed the subcommand.
  //    When complete, reading 0x3E/0x3F returns what we wrote.
  while (true) {
    Wire.beginTransmission(BQ_ADDR);
    Wire.write(0x3E);
    Wire.endTransmission(false);
    Wire.requestFrom(BQ_ADDR, (uint8_t)2);
    uint8_t rLo = Wire.read();
    uint8_t rHi = Wire.read();
    if (rLo == subLo && rHi == subHi) break;
    delay(2);
  }

  // 3) Read length from 0x61 (optional, but good sanity check)
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(0x61);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ_ADDR, (uint8_t)1);
  uint8_t len = Wire.read();
  // len = total bytes (0x3E,0x3F, data bytes, 0x60,0x61).
  // For a 2-byte parameter this is typically 0x06.

  // 4) Read buffer starting at 0x40; first 2 bytes are the parameter
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(0x40);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ_ADDR, (uint8_t)2);
  uint8_t valLo = Wire.read();
  uint8_t valHi = Wire.read();

  return (uint16_t)valLo | ((uint16_t)valHi << 8);
}

// Helper: write a data-memory word at 16‑bit address (subcommand 0x3E/0x3F)
void bqWriteDataMemWord(uint16_t addr, uint16_t value) {
  uint8_t subLo = addr & 0xFF;        // 0x09 for 0x9309
  uint8_t subHi = (addr >> 8) & 0xFF; // 0x93 for 0x9309
  
  // 1) Write subcommand to 0x3E/0x3F
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(0x3E); Wire.write(subLo); Wire.write(subHi);
  Wire.endTransmission();
  
  // 2) Data buffer: 2 bytes at 0x40-0x41
  uint8_t dataLo = value & 0xFF;  // LSB first!
  uint8_t dataHi = (value >> 8);  
  
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(0x40); Wire.write(dataLo); Wire.write(dataHi);
  Wire.endTransmission();
  
  // 3) CRITICAL: Correct checksum/length calculation
  uint8_t sum = subLo + subHi + dataLo + dataHi;  // Sum ALL bytes
  uint8_t checksum = ~sum & 0xFF;                 // 8-bit invert
  uint8_t length = 6;                            // 2(subcmd)+2(data)+2(chk/len)
  
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(0x60); Wire.write(checksum); Wire.write(length);
  Wire.endTransmission();
}

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ_ADDR, 1);
  return Wire.read();
}


// Wait until CFGUPDATE flag is set/clear in 0x12 Battery Status
void waitCfgUpdate(bool targetSet) {
  while (true) {
    uint16_t bs = bqReadWord(0x12); // Battery Status
    bool cfg = bs & 0x0001;         // bit0 = CFGUPDATE
    if (cfg == targetSet) break;
    delay(10);
  }
}

void enterFullAccess() {
  // You must send your actual UNSEAL and FULLACCESS keys here using 0x3E/0x3F,
  // as described in TRM section “Device Security”.
}

bool isDischarging(void) {
  byte regData = (byte)directCommand(0x7F);
  if(regData & 0x04) {
    Serial.println("[+] Discharging FET -> ON");
    return true;
  }
  Serial.println(F("[+] Discharging FET -> OFF"));
  return false;
}


void readSubcommand(uint16_t subcmd, uint8_t *buffer) {

  // 1–2. Write subcommand
  writeReg(0x3E, subcmd & 0xFF);        // LSB
  writeReg(0x3F, (subcmd >> 8) & 0xFF); // MSB

  // 3. Wait until command completes
  while (true) {
    uint8_t l = readReg(0x3E);
    uint8_t h = readReg(0x3F);
    if (l == (subcmd & 0xFF) && h == ((subcmd >> 8) & 0xFF))
      break;
  }

  // 4. Read length
  uint8_t length = readReg(0x61);

  // 5. Read buffer
  for (uint8_t i = 0; i < (length - 4); i++) {
    buffer[i] = readReg(0x40 + i);
  }

  // 6. Optional: verify checksum
  uint8_t checksum = readReg(0x60);
}

void writeSubcommand(uint16_t subcmd, uint8_t *data, uint8_t dataLen) {

  // 1–2. Write subcommand
  writeReg(0x3E, subcmd & 0xFF);
  writeReg(0x3F, (subcmd >> 8) & 0xFF);

  // 3. Write data to buffer
  uint8_t sum = (subcmd & 0xFF) + ((subcmd >> 8) & 0xFF);

  for (uint8_t i = 0; i < dataLen; i++) {
    writeReg(0x40 + i, data[i]);
    sum += data[i];
  }

  // 4. Compute checksum
  uint8_t checksum = ~sum;

  // 5. Compute total length
  uint8_t length = dataLen + 4;

  // Write checksum and length (must be written)
  writeReg(0x60, checksum);
  writeReg(0x61, length);
}

enum states {INIT,buttonAPressed, buttonBPressed,DCHG, DCHG_done ,CHG, CHG_done} state;

void tick(){
  
  int buttonA;
  int buttonB;
  int batt_cell[10];
  int16_t batt_Curr = directCommand(0x3A)/3;
  for (int i =0; i<num_cell; i++){batt_cell[i]=directCommand(CELL_NO_TO_ADDR(i+1));}
  
  switch(state){
  
    case INIT:
     buttonA = digitalRead(buttonA_pin);
     if(buttonA == 1){state = buttonAPressed;}
     break;
     
    case buttonAPressed:
     buttonA = digitalRead(buttonA_pin);
     Serial.println("button A pressed");
     if(buttonA == 0){state = DCHG;}
     break;
     
    case buttonBPressed:
     buttonA = digitalRead(buttonA_pin);
     Serial.println("button B pressed");
     if(buttonA == 0){state = INIT;}
     break;
     
    case DCHG:
      buttonA = digitalRead(buttonA_pin);
      if(buttonA == 1){state = buttonBPressed;}
      for (int i =0; i<num_cell; i++){ if(batt_cell[i]< 2700){state = DCHG_done;}}
     
     break;
    case DCHG_done:
     buttonA = digitalRead(buttonA_pin);
     if(buttonA == 1){state = buttonBPressed;}
     //if(batt_Curr < 0){state = CHG;}
     
     break;
    case CHG:
     for (int i =0; i<num_cell; i++){ if(batt_cell[i]> 3700){state = INIT;}}
     break;
    case CHG_done:
     break;
  }
  switch(state){
     case INIT:
      sendSubcommand(0x0095);
      Serial.println("INIT");
      digitalWrite(greenLED, 0);
      digitalWrite(redLED, 0);
     break;
    case DCHG:
     sendSubcommand(0x0096);
     Serial.println("DCHG");
     digitalWrite(greenLED, 1);
     break;
    case DCHG_done:
     sendSubcommand(0x0095);
     Serial.println("DCHG done");
     digitalWrite(greenLED, 0);
     digitalWrite(redLED, 1);
     break;
    case CHG:
     sendSubcommand(0x0096);
     Serial.println("CHG");
     digitalWrite(redLED, !(isDischarging()));
     digitalWrite(greenLED, isDischarging());
     digitalWrite(greenLED, !(isDischarging()));
     break;
    case CHG_done:
     break;
  }
  
}


void setup() {


  Wire.begin();
  Serial.begin(115200);
  delay(10);
  
  pinMode(buttonA_pin, INPUT);
  pinMode(buttonB_pin, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  // 1) Enter CONFIGUPDATE mode: subcommand 0x0090
  
  sendSubcommand(0x0090);
  waitCfgUpdate(true);

  // 3) Write Vcell Mode = 0x0007 at data memory address 0x9304
  bqWriteDataMemWord(0x9304, 0x03FF); //battery cell configuration
  bqWriteDataMemWord(0x9335, 0x0003); //auto cell balancing active
  bqWriteDataMemWord(0x9343, 0x0050); //FET ENABLE
  bqWriteDataMemWord(0x9308, 0x000D); //Fet control Enable
  bqWriteDataMemWord(0x9309, 0x0001); //Charge pump enable
  bqWriteDataMemWord(0x9336, 0x0000);  //configure minimum  temperature
  //bqWriteDataMemWord(0x933F, 0x0E74); //auto cell balancing - relax
  
  // 4) Exit CONFIGUPDATE: subcommand 0x0092
  sendSubcommand(0x0092);
  waitCfgUpdate(false);
}


void loop() {
  // normal operation

 // bqWriteDataMemWord(0x0084, 0x0ED8);
  //bqWriteDataMemWord(0x0083, 0x0200);
//  int16_t TS1 = directCommand(0x70);
 // int16_t TS3 = directCommand(0x74);
 // int16_t data =bqReadDataMemWord(0x0085);
//  int32_t currData = directCommand(0x3A)/3;
 // Serial.println('1');
  tick();
  delay(500);
}