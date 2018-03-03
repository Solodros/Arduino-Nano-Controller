#include <SPI.h>
#include <EEPROM.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <VescUart.h>

//#define DEBUG
#ifdef DEBUG
#include <printf.h>
#define DEBUG_PRINT(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#endif

// Transmit and receive package
struct package {
  uint8_t type;     // | Normal:0  | Setting:1
  uint8_t throttle; // | Throttle  |
  uint8_t trigger;  // | Trigger   |
  uint32_t id;
};

#define NORMAL 0
#define SETTING 1

// Defining constants to hold the special settings, so it's easy changed thoughout the code
#define TRIGGER 0
#define MODE    7
#define ADDRESS 14

// When receiving a "type: 1" package save the next transmission (a new setting) in this struct
struct settingPackage {
  uint8_t id;
  uint8_t setting;
  uint64_t value;
};

// Defining struct to handle callback data (auto ack)
struct callback {
  float dutyNow;
  long rpm;
  float inpVoltage;
  float ampHours;
  long tachometerAbs;
};

// Defining struct to handle receiver settings
struct settings {
  uint8_t triggerMode; // Trigger mode
  uint8_t controlMode; // PWM, PWM & UART or UART only
  uint64_t address;    // Listen on this address
};

const uint8_t numOfSettings = 3;
// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
  {0, 1, 2}, // 0: Safety On | 1: Safety Off       | 2: Cruise Control
  {1, 0, 2}, // 0: PPM only   | 1: PPM and UART | 2: UART only
  { -1, 0, 0} // No validation for address in this manner
};

struct bldcMeasure uartData;
struct callback returnData;
struct package remPackage;
struct settingPackage setPackage;
struct settings rxSettings;

// Define default 8 byte address
const uint64_t defaultAddress = 0xE8E8F0F1E9LL;
const uint8_t defaultChannel = 108;
uint32_t timeoutTimer = 0;
bool recievedData = false;

// Current mode of receiver - 0: Connected | 1: Timeout | 2: Updating settings
uint8_t statusMode = 0;

#define CONNECTED 0
#define TIMEOUT 1
#define UPDATING 2
#define RESETTING 3

// Last time data was pulled from VESC
unsigned long lastUartPull;

// Address reset button
unsigned long resetButtonTimer;
bool resetButtonState = LOW;

// Status blink LED
unsigned long lastStatusBlink;
bool statusBlinkFlag = LOW;
short iBlink=0;
const int fancyBlink[][8] = {
    // Connected
    {100, 50, 50, 50, 50, 100, 100, 50},
    // Timeout / Disconnected
    {500, 500, 500, 500, 500, 500, 500, 500},

    // On update or changing setting.
    {25, 25, 25, 25, 25, 25, 25, 25},

    // On reset
    {100, 100, 100, 100, 1000, 100, 5000, 500}

  };

const uint8_t defaultThrottle = 127;
const short timeoutMax = 100;

// Defining RX pins
const uint8_t CE = 9;
const uint8_t CS = 10;
const uint8_t statusLedPin = 4;
const uint8_t throttlePin = 5;
const uint8_t resetAddressPin = 6;

// Initiate RF24 class
RF24 radio(CE, CS);

#define SERIALIO Serial

const int EEPROM_ADDR_FLOW = 0;

void setup()
{
#ifdef DEBUG
  printf_begin();
  // Open serial communications USB:
  Serial.begin(115200);
#else
  // Open serial VESC data
  //Setup UART port
  SetSerialPort(&SERIALIO);
  SERIALIO.begin(115200);
#endif


  pinMode(throttlePin, OUTPUT);
  pinMode(statusLedPin, OUTPUT);
  pinMode(resetAddressPin, INPUT_PULLUP);

  // Set default throttle in startup
  analogWrite(throttlePin, defaultThrottle);

  //setDefaultEEPROMSettings();
  loadEEPROMSettings();
  initiateReceiver();
}

void loop()
{

  statusBlink();

  while (radio.available()){
    radio.read( &remPackage, sizeof(remPackage));
    if ( remPackage.type <= 2 ) {
      timeoutTimer = millis();
      recievedData = true;
    }else{ radio.read( &setPackage, sizeof(setPackage)); }
    break;
  }

  if (recievedData == true){
    statusMode = CONNECTED;

    if ( remPackage.type == NORMAL ) {

      updateThrottle( remPackage.throttle );

      if ( rxSettings.controlMode != 0 ) {
        getUartData();
        radio.writeAckPayload(1, &returnData, sizeof(returnData));
      }

    }else if( remPackage.type == SETTING ){
      statusMode = UPDATING;
      acquireSetting();
    }
    recievedData = false;
  }





  /* Begin timeout handling */
  if ( timeoutMax <= ( millis() - timeoutTimer ) ) {
    // No speed is received within the timeout limit.
    updateThrottle( defaultThrottle );
    timeoutTimer = millis();
    statusMode = TIMEOUT;
  }
  /* End timeout handling */
}

void statusBlink() {
  short mode=0;

  switch (statusMode) {
    case CONNECTED:
      mode=0;
      break;

    case TIMEOUT:
      mode=1;
      break;

    case UPDATING:
      mode=2;
      break;

    case RESETTING:
      mode=3;
      break;
  }

  if ((millis() - lastStatusBlink) > fancyBlink[mode][iBlink]){
    if( statusBlinkFlag == LOW){
      statusBlinkFlag = HIGH;
      lastStatusBlink = millis();
    }else{
      statusBlinkFlag = LOW;
      lastStatusBlink = millis();
    }
    iBlink++;
  }

  digitalWrite(statusLedPin, statusBlinkFlag);

  if(iBlink >= (sizeof(fancyBlink[mode])/sizeof(int))){ iBlink=0; }
}

void acquireSetting() {
  uint8_t setting;
  uint64_t value;
  bool flag[4] = {false,false,false,false};
  short timer = 0;
  
  if(remPackage.id==1){
    remPackage.id = 2;
    radio.writeAckPayload(1, &remPackage, sizeof(remPackage));
    flag[0]=true;
  }

  while(timer<10000){
    while(radio.available()){
      radio.read( &setPackage, sizeof(setPackage));
      if(setPackage.id == 1){
        setPackage.id=2;
        setting = setPackage.setting;
        value = setPackage.value;
        radio.writeAckPayload(1, &setPackage, sizeof(setPackage));
        flag[2]=true;
      }else if(setPackage.id == 3 && setPackage.setting==setting && setPackage.value==value){
        setPackage.id=4;
        radio.writeAckPayload(1, &setPackage, sizeof(setPackage));
        flag[3]=true;
        break;
      }else{

      }
      flag[1]=true;
    }
    timer++;
  }

if(flag[0]==true){DEBUG_PRINT("ERROR TX 01");}
if(flag[1]==true){DEBUG_PRINT("ERROR TX 02");}
if(flag[2]==true){DEBUG_PRINT("ERROR TX 03");}
if(flag[3]==true){DEBUG_PRINT("ERROR TX 04");}

  if(flag[0]==true && flag[1]==true && flag[2]==true && flag[3]==true){
    updateSetting(setting, value);
  }
  DEBUG_PRINT("END OF SETTING");
} /*END OF acquireSetting*/


void initiateReceiver() {
  if (rxSettings.address == 0) { rxSettings.address = defaultAddress; }
  radio.begin();
  radio.setChannel(defaultChannel);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, rxSettings.address);
  radio.startListening();
  radio.setRetries(15, 15);

  #ifdef DEBUG
    radio.printDetails();
  #endif
}

// Update a single setting value
void updateSetting( uint8_t setting, uint64_t value)
{
  // Map remote setting indexes to receiver settings
  switch ( setting ) {
    case TRIGGER: setting = 0; break;  // TriggerMode
    case MODE: setting = 1; break;  // ControlMode
    case ADDRESS: setting = 2; break; // Address
  }
  setSettingValue( setting, value);
  updateEEPROMSettings();

  // The address has changed, we need to reinitiate the receiver module
  if (setting == 2) {
    DEBUG_PRINT("Connection Address Refresh.");
    initiateReceiver();
  }
}

void updateThrottle( uint8_t throttle )
{
  switch ( rxSettings.controlMode )
  {
    // PPM
    case 0:
      // Write the PWM signal to the ESC (0-255).
      analogWrite(throttlePin, throttle);
      break;

    // PPM and UART
    case 1:
      // Write the PWM signal to the ESC (0-255).
      analogWrite(throttlePin, throttle);
      break;

    // UART
    case 2:
      // Update throttle with UART
      break;
  }
}

void getUartData()
{
  if ( millis() - lastUartPull >= 250 ) {

    lastUartPull = millis();

    // Only get what we need
    if ( VescUartGetValue(uartData) )
    {
      returnData.dutyNow        = uartData.dutyNow;
      returnData.ampHours       = uartData.ampHours;
      returnData.inpVoltage     = uartData.inpVoltage;
      returnData.rpm            = uartData.rpm;
      returnData.tachometerAbs  = uartData.tachometerAbs;
    }
    else
    {
      returnData.dutyNow        = 0;
      returnData.ampHours       = 0.0;
      returnData.inpVoltage     = 0.0;
      returnData.rpm            = 0;
      returnData.tachometerAbs  = 0;
    }
  }
}

String uint64ToString(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  if (part1 == 0) {
    return String(part2, DEC);
  }

  return String(part1, DEC) + String(part2, DEC);
}

String uint64ToAddress(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  return String(part1, HEX) + String(part2, HEX);
}

// Settings functions

void setDefaultEEPROMSettings()
{
  clearEEPROM();
  DEBUG_PRINT("Calling setDefaultEEPROMSettings()");
  for (int i = 0; i < numOfSettings; i++) {
    setSettingValue(i, settingRules[i][0]);
  }
  rxSettings.address = defaultAddress;
  updateEEPROMSettings();
}

void loadEEPROMSettings()
{
  DEBUG_PRINT("** START loadEEPROMSettings **");
  bool rewriteSettings = false;
  // Load settings from EEPROM to custom struct
  EEPROM.get(EEPROM_ADDR_FLOW, rxSettings);

  // Loop through all settings to check if everything is fine
  for ( int i = 0; i < numOfSettings; i++ ) {
    int val = getSettingValue(i);

    // If setting default value is -1, don't check if its valid
    if ( settingRules[i][0] != -1 )
    {
      if ( !inRange( val, settingRules[i][1], settingRules[i][2] ) )
      {
        // Setting is damaged or never written. Rewrite default.
        rewriteSettings = true;
        setSettingValue(i, settingRules[i][0] );
      }
    }
  }

  if (rewriteSettings == true) {
    updateEEPROMSettings();
  }
  DEBUG_PRINT("** END loadEEPROMSettings **");
}

// Write settings to the EEPROM
void updateEEPROMSettings() {
  DEBUG_PRINT("** updateEEPROMSettings **");
  EEPROM.put(EEPROM_ADDR_FLOW, rxSettings);
}

// Set a value of a specific setting by index.
void setSettingValue(int index, uint64_t value)
{
  switch (index) {
    case 0: rxSettings.triggerMode = value; break;
    case 1: rxSettings.controlMode = value; break;
    case 2: rxSettings.address = value;     break;
  }
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(uint8_t index)
{
  int value;
  switch (index) {
    case 0: value = rxSettings.triggerMode; break;
    case 1: value = rxSettings.controlMode; break;
  }
  return value;
}

bool inRange(int val, int minimum, int maximum)
{
  return ((minimum <= val) && (val <= maximum));
}

void clearEEPROM() {
  DEBUG_PRINT("Calling clearEEPROM()");
  for (int i = 0 ; i < (EEPROM.length() + EEPROM_ADDR_FLOW) ; i++) {
    EEPROM.write(i, 0);
  }
}
