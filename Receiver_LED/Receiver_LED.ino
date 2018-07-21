#include <SPI.h>
#include <EEPROM.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <VescUart.h>
#include <Servo.h>
Servo servoPin;

//#define DEBUG
#ifdef DEBUG
#include <printf.h>
#define DEBUG_PRINT(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#endif

#define NORMAL 0
#define SETTING 1

// Defining constants to hold the special settings, so it's easy changed thoughout the code
#define TRIGGER 1
#define ADDRESS 15
#define STEPPER 8

// When receiving a "type: 1" package save the next transmission (a new setting) in this struct
struct settingPackage {
  uint8_t id;
  uint8_t setting;
  uint64_t value;
};

// Defining struct to handle callback data (auto ack)
struct packageRX { // RX TO TX
  float dutyNow;
  long rpm;
  float inpVoltage;
  float ampHours;
  long tachometerAbs;

  bool cruise;
  short throttle_set;
};
struct packageTX {
  uint8_t type;     // | Normal:0  | Setting:1
  short throttle; // | Throttle  |
  bool trigger;  // | Trigger   |
  uint8_t id;
  uint8_t controlMode;

  uint8_t stepper;
  short Kp;
  short Ki;
  short Kd;
  short ratePing;
};
// Defining struct to handle receiver settings
struct settings {
  uint8_t controlMode; // Trigger mode
  uint64_t address;    // Listen on this address
  uint8_t stepper;
};

const uint8_t numOfSettings = 3;
// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
  {1, 0, 3}, // 0: S-ON | 1: S-OFF | 2: CRUISE | 3: BEGINNER
  { -1, 0, 0}, // No validation for address in this manner
  { 10, 0, 100} // #8 STEPPER SPEED
};

struct bldcMeasure uartData;
struct packageRX returnData;
struct packageTX txPacket;
struct settingPackage setPackage;
struct settings rxSettings;


// Define default 8 byte address
const uint64_t defaultAddress = 0xE8E8F0F1E9LL;
const uint8_t defaultChannel = 108;
uint32_t timeoutTimer = 0;
bool recievedData = false;
unsigned long lastPing;

// Current mode of receiver - 0: Connected | 1: Timeout | 2: Updating settings
uint8_t statusMode = 0;

#define CONNECTED 0
#define TIMEOUT 1
#define UPDATING 2
#define RESETTING 3

// Last time data was pulled from VESC
unsigned long lastUartPull;
bool UartUpdated = false;

// Address reset button
unsigned long resetButtonTimer;
bool resetButtonState = LOW;

// Status blink LED
unsigned long lastStatusBlink;
bool statusBlinkFlag = LOW;
short iBlink=0;
const int fancyBlink[][8] = {
    // Connected
    {25, 25, 25, 25, 25, 25, 25, 25},
    // Timeout / Disconnected
    {100, 500, 100, 500, 500, 500, 500, 500},

    // On update or changing setting.
    {50, 150, 50, 150, 50, 150, 50, 150},

    // On reset
    {100, 100, 100, 100, 1000, 100, 5000, 500}

  };

short safetyThrottle = 1500;
bool safetyActive = false;
const short defaultThrottle = 1500;
const short timeoutMax = 100;

// Defining RX pins
const uint8_t CE = 9;
const uint8_t CS = 10;
const uint8_t statusLedPin = 3;
const uint8_t GreenLedPin = 6;
const uint8_t AmberLedPin = 7;
const uint8_t RedLedPin = 8;
const uint8_t throttlePin = 5;
const uint8_t resetAddressPin = 2;


// Initiate RF24 class
RF24 radio(CE, CS);

#define SERIALIO Serial

const int EEPROM_ADDR_FLOW = 0;

// Cruise Control VARS
long rpm_set;
long err;
long err_T;
long err_P;
float throttle_adjustment;
float Kp, Ki, Kd;

// Battery Indicate LEDs
int BatteryPercent = 100;
short countBlink=0;
const uint8_t BatteryCells = 12;
const float minVoltage = 3.2;
const float maxVoltage = 4.2;
unsigned long BatteryLastBlinkLed;
const int BatteryBlink[4][8]={
{250,500,250,500,250,500,150,1000}, // x4
{250,500,250,500,250,1000,150,1000}, // x3
{250,1000,250,150,150,150,150,150}, // x2
{1000,1000,1000,1000,1000,1000,1000,1000} // x2
};
bool BatteryLED[3]={LOW,LOW,LOW};

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
  pinMode(GreenLedPin, OUTPUT);
  pinMode(AmberLedPin, OUTPUT);
  pinMode(RedLedPin, OUTPUT);
  digitalWrite(GreenLedPin, HIGH);
  digitalWrite(AmberLedPin, HIGH);
  digitalWrite(RedLedPin, HIGH);

  // Set default throttle in startup
  servoPin.attach(throttlePin);
  servoPin.writeMicroseconds(defaultThrottle);

  //setDefaultEEPROMSettings();
  loadEEPROMSettings();

  initiateReceiver();
}

void loop()
{

  statusBlink();
  statusBattery();

  while (radio.available()){
    radio.read( &txPacket, sizeof(txPacket));
    if ( txPacket.type <= 1 ) {
      timeoutTimer = millis();
      recievedData = safetyActive = true;
      Kp=0.0001*txPacket.Kp; Ki=0.00001*txPacket.Ki; Kd=0.0001*txPacket.Kd;
      if(txPacket.ratePing<=0){ txPacket.ratePing=250; }

    }else{ radio.read( &setPackage, sizeof(setPackage)); }
    break;
  }

  if (recievedData == true){
    statusMode = CONNECTED;

    if ( txPacket.type == NORMAL ) {
      getUartData();
      /*
      controlMode
      0 = S-ON
      1 = S-OFF
      2 = CRUISE
      3 = BEGINNER 20km limit
      */
      if(rxSettings.controlMode==2 && txPacket.trigger){
        // Check if is changing throttle.
        if(!((defaultThrottle+5) > txPacket.throttle && (defaultThrottle-5) < txPacket.throttle)){
          rpm_set = (returnData.rpm);
          cruise_PID();
          servoPin.writeMicroseconds(txPacket.throttle);
        }else{
          if(returnData.cruise==false){ rpm_set = (returnData.rpm); }
          cruise_PID();
          servoPin.writeMicroseconds(returnData.throttle_set);
        }
        returnData.cruise=true;
      }else{
        if(rxSettings.controlMode==2){
          rpm_set = (returnData.rpm);
          cruise_PID();
          returnData.cruise=false;
        }

        servoPin.writeMicroseconds(txPacket.throttle);
      }

      // feeding packets back to TX.
      if(UartUpdated){ radio.writeAckPayload(1, &returnData, sizeof(returnData)); }

    }else if( txPacket.type == SETTING ){
      statusMode = UPDATING;
      if(txPacket.id>0){
        acquireSetting();
      }else{
        if((millis() - lastPing)>txPacket.ratePing){
          lastPing=millis();
          radio.writeAckPayload(1, &txPacket, sizeof(txPacket));
        }
      }
    }
    recievedData = false;
  }





  /* Begin timeout handling */
  if ( timeoutMax <= ( millis() - timeoutTimer ) ) {
    // No speed is received within the timeout limit.
    timeoutTimer = millis();
    statusMode = TIMEOUT;
  }

  if(statusMode==TIMEOUT){
    if(safetyActive==true){
      safetyThrottle-5;
      if(safetyThrottle<1000){ safetyThrottle=1000; }
      servoPin.writeMicroseconds(safetyThrottle);
    }else{
      servoPin.writeMicroseconds(defaultThrottle);
    }
  }else{
    safetyThrottle=defaultThrottle;
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
    }else{
      statusBlinkFlag = LOW;
    }

    lastStatusBlink = millis();
    iBlink++;
  }

  digitalWrite(statusLedPin, statusBlinkFlag);

  if(iBlink >= (sizeof(fancyBlink[mode])/sizeof(int))){ iBlink=0; }
}

void statusBattery(){
  uint8_t i_status = 0;
  uint8_t i_led = 0;
  BatteryPercent = (((returnData.inpVoltage/BatteryCells-minVoltage)*(100/(maxVoltage-minVoltage))));

  if(BatteryPercent>=90){        i_status=0; i_led=0; // GREEN x4
  }else if(BatteryPercent>=80){  i_status=1; i_led=0;// GREEN x3
  }else if(BatteryPercent>=70){  i_status=2; i_led=0;// GREEN x2
  }else if(BatteryPercent>=60){  i_status=0; i_led=1;// AMBER x4
  }else if(BatteryPercent>=50){  i_status=1; i_led=1;// AMBER x3
  }else if(BatteryPercent>=40){  i_status=2; i_led=1; // AMBER x2
  }else if(BatteryPercent>=30){  i_status=0; i_led=2;// RED x4
  }else if(BatteryPercent>=20){  i_status=1; i_led=2;// RED x3
  }else if(BatteryPercent>=10){  i_status=2; i_led=2;// RED x2
  }else{ i_status=3; i_led=2; }

 if(BatteryPercent>=70){ BatteryLED[1] = BatteryLED[2] = HIGH;
 }else if(BatteryPercent>=40){ BatteryLED[0]=LOW; BatteryLED[2]=HIGH;
 }else if(BatteryPercent>=10){  BatteryLED[0] = BatteryLED[1] = LOW;
 }else{ BatteryLED[0] = BatteryLED[1] = LOW; }

 if ((millis() - BatteryLastBlinkLed) > BatteryBlink[i_status][countBlink]){
     if(BatteryLED[i_led] == LOW){
       BatteryLED[i_led] = HIGH;
     }else{
       BatteryLED[i_led] = LOW;
     }
     BatteryLastBlinkLed = millis();
     countBlink++;
 }

  digitalWrite(GreenLedPin, BatteryLED[0]);
  digitalWrite(AmberLedPin, BatteryLED[1]);
  digitalWrite(RedLedPin, BatteryLED[2]);

  if(countBlink>=(sizeof(BatteryBlink[i_status])/sizeof(int))){ countBlink = 0; }
}

void acquireSetting() {
  uint8_t setting;
  uint64_t value;
  bool flag[3] = {false,false,false};

  if(txPacket.id==1){
    txPacket.id = 2;
    radio.writeAckPayload(1, &txPacket, sizeof(txPacket));
    flag[0]=true;
  }
  unsigned short timer=0;
  while(timer<6000){
    if(radio.available()){
      radio.read( &setPackage, sizeof(setPackage));
      if(setPackage.id==1 || setPackage.id==2){
        setPackage.id=2;
        setting = setPackage.setting;
        value = setPackage.value;
        radio.writeAckPayload(1, &setPackage, sizeof(setPackage));
        //delay(15);
        flag[2]=true;
      }
      flag[1]=true;
    }
    timer++;
  }

  if(flag[0]==true && flag[1]==true && flag[2]==true){
    updateSetting(setting, value);
  }else{
    initiateReceiver();
  }
  DEBUG_PRINT("END OF SETTING");
} /*END OF acquireSetting*/


void initiateReceiver() {
  if (uint64ToAddress(rxSettings.address) == "ffffffffffffffff") { rxSettings.address = defaultAddress; }
  radio.begin();
  radio.setChannel(defaultChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, rxSettings.address);
  radio.startListening();
  radio.setRetries(15, 15);
}

// Update a single setting value
void updateSetting( uint8_t setting, uint64_t value)
{
  // Map remote setting indexes to receiver settings
  switch ( setting ) {
    case TRIGGER: setting = 0; break;  // TriggerMode
    case ADDRESS: setting = 1; break; // Address
    case STEPPER: setting = 2; break; // STEPPER
  }
  setSettingValue( setting, value);
  updateEEPROMSettings();

  // The address has changed, we need to reinitiate the receiver module
  if (setting == 1) {
    DEBUG_PRINT("Connection Address Refresh.");
    initiateReceiver();
  }
}

void getUartData()
{
  UartUpdated = false;
  if ((millis() - lastUartPull) >= txPacket.ratePing) {

    lastUartPull = millis();
    UartUpdated = true;

    // Only get what we need
    if ( VescUartGetValue(uartData) )
    {
      returnData.dutyNow        = uartData.dutyNow;
      returnData.ampHours       = uartData.ampHours;
      returnData.inpVoltage     = uartData.inpVoltage;
      returnData.rpm            = (uartData.rpm);
      returnData.tachometerAbs  = (uartData.tachometerAbs);
    }
    else
    {
      returnData.dutyNow        = 0.0;
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
}

// Write settings to the EEPROM
void updateEEPROMSettings() {
  EEPROM.put(EEPROM_ADDR_FLOW, rxSettings);
}

// Set a value of a specific setting by index.
void setSettingValue(int index, uint64_t value)
{
  switch (index) {
    case 0: rxSettings.controlMode = value; break;
    case 1: rxSettings.address = value;     break;
  }
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(uint8_t index)
{
  int value;
  switch (index) {
    case 0: value = rxSettings.controlMode; break;
  }
  return value;
}

bool inRange(int val, int minimum, int maximum)
{
  return ((minimum <= val) && (val <= maximum));
}

void clearEEPROM() {
  for (int i = 0 ; i < (EEPROM.length() + EEPROM_ADDR_FLOW) ; i++) {
    EEPROM.write(i, 0);
  }
}

void cruise_PID(){
  if(statusMode==TIMEOUT){
    returnData.throttle_set=err=err_T=err_P=throttle_adjustment=0;
  }else if (UartUpdated){
    err = ((returnData.rpm)-rpm_set);
    err_T+=err;
    throttle_adjustment = ((Kp*err) + (Ki*err_T) + (Kd*(err - err_P)));
    err_P = err;
  }
  throttle_adjustment = constrain(1500 - (int)throttle_adjustment, 1000, 2000);

  if((((returnData.throttle_set-txPacket.stepper) < throttle_adjustment) || (throttle_adjustment < (returnData.throttle_set+txPacket.stepper)))){
    returnData.throttle_set = throttle_adjustment;
  }else if(throttle_adjustment > returnData.throttle_set){
    returnData.throttle_set++;
  }else{
    returnData.throttle_set--;
  }
}
