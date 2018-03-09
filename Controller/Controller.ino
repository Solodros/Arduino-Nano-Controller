#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RF24.h>
#include <VescUart.h>

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.println (x)
#include "printf.h"
#else
#define DEBUG_PRINT(x)
#endif

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const unsigned char logo[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01,
  0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19,
  0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37,
  0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01,
  0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07,
  0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char transmittingIcon[] = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
  0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char connectedIcon[] = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
  0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char noconnectionIcon[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

struct callback {
  float dutyNow;
  long rpm;
  float inpVoltage;
  float ampHours;
  long tachometerAbs;
};
struct package {
  uint8_t type;     // | Normal:0  | Setting:1
  short throttle; // | Throttle  |
  uint8_t trigger;  // | Trigger   |
  uint32_t id;
};
struct settingPackage {
  uint8_t id;
  uint8_t setting;
  uint64_t value;
};
struct settings {
  uint8_t triggerMode;
  uint8_t batteryType;
  uint8_t batteryCells;
  uint8_t motorPoles;
  uint8_t motorPulley;
  uint8_t wheelPulley;
  uint8_t wheelDiameter;
  uint8_t controlMode;
  short minHallValue;
  short centerHallValue;
  short maxHallValue;
  uint8_t stepCruise;
  short minPWM;
  short maxPWM;
  uint64_t address;
  uint8_t rate;
  uint8_t exit;
};

float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

uint8_t currentSetting = 0;
const uint8_t numOfSettings = 18;

const short settingRules[numOfSettings][3] {
  {1, 0, 2}, // 0: Safety On or 1: Safety Off 2: Cruise Controll
  {0, 0, 1}, // 0 Li-ion & 1 LiPo
  {10, 0, 12},
  {14, 0, 250},
  {14, 0, 250},
  {66, 0, 250},
  {203, 0, 250},
  {1, 0, 2}, // 0: PPM only, 1: PPM and UART  or 2: UART only
  { 1, 0, 10}, // CRUISE CONTROLL
  {20, 0, 1023},
  {493, 0, 1023},
  {950, 0, 1023},
  { 1000, 0, 1000}, // MIN PWM
  { 2000, 0, 2300}, // MAX PWM
  { -1, 0, 0}, // No validation for pipe address (not really possible this way)
  { -1, 0, 0}, // No validation for default address
  { 50, 1, 50}, // Timing TX rate by millionsecond
  { 0, 0, 1} // EXIT
};

#define TRIGGER 0
#define MODE    7
#define ADDRESS 14
#define RESET 15

struct callback returnData;
struct package remPackage;
struct settingPackage setPackage;
struct settings txSettings;

const uint8_t triggerPin = 4;
const uint8_t batteryMeasurePin = A2;
const uint8_t hallSensorPin = A3;

const float minVoltage = 3.2;
const float maxVoltage = 4.1;
const float refVoltage = 5.0;

short hallValue;
short throttle;
uint8_t hallCenterMargin = 4;
uint8_t hallMenuMargin = 100;
short hallCenterNoise = 1500;

const uint64_t defaultAddress = 0xE8E8F0F1E9LL;
const uint8_t defaultChannel = 108;
unsigned long lastTransmission;
bool connected = false;
uint8_t failCount;

String tempString;
unsigned long lastSignalBlink;
unsigned long lastDataRotation;

bool changeSettings = false;
bool changeSelectedSetting = false;
bool settingsLoopFlag = false;
bool settingsChangeFlag = false;

bool signalBlink = false;

short CruiseValue;
bool CruiseActivated = false;
bool CruiseSafety = false;
#define CRUISE 8
#define MINPWM 12
#define MAXPWM 13
bool safety[3] = {false,false,false};

RF24 radio(9, 10);

void setup() {
	#ifdef DEBUG
	  Serial.begin(115200);
	  printf_begin();
	#endif

  //setDefaultEEPROMSettings();
  loadEEPROMSettings();

  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
  pinMode(batteryMeasurePin, INPUT);

  u8g2.begin();
  drawStartScreen();

  getThrottlePosition();
  if(isTrigger()){if (hallValue < (txSettings.minHallValue + hallMenuMargin)){changeSettings=true;drawTitleScreen(F("Remote Settings"));}}
  CruiseValue = txSettings.centerHallValue;

  initiateTransmitter();
}

void loop() {
  getThrottlePosition();


  if (changeSettings == true) {
    controlSettingsMenu();
    safety[0] = false;

  }else{
    remPackage.type = 0;
    remPackage.trigger = isTrigger();

    if (!safety[0] && isNeutral()) { safety[0] = true; }

    if(safety[0]){
      if (txSettings.triggerMode <= 0){
        if(isTrigger()){
          if(isNeutral()){
            safety[1]=true;
          }else if(safety[2]==true){
            safety[1]=safety[2]=false;
            CruiseValue=txSettings.centerHallValue;
          }

          if(safety[1]==true){
            CruiseValue=hallValue;
            remPackage.throttle = throttle;
          }else{setNeutral();}

        }else{
          if(safety[1]==true){
            cruise_stepper();
            safety[2]=true;
            remPackage.throttle = throttle;
            if(hallCenterNoise==throttle){
              safety[1]=safety[2]=false;
            }

          }else if(!isNeutral()){
            safety[1]=false;
            setNeutral();
          }
        }

      }else{
          if(txSettings.triggerMode==2){ cruise_mode(); }
          else{ remPackage.throttle = throttle; }
      }
    }else{
      setNeutral();
    }
    transmitToReceiver();
  }

  // Call function to update display
  updateMainDisplay();
}

// When called the throttle and trigger will be used to navigate and change settings
void controlSettingsMenu() {
short val;
  // If thumbwheel is in top position
  if (hallValue >= (txSettings.centerHallValue + hallMenuMargin) && settingsLoopFlag == false) {
    // Up
    if (changeSelectedSetting == true) {
      if (settingRules[currentSetting][0] != -1) {

        if(currentSetting==MINPWM || currentSetting==MAXPWM){
          val = getSettingValue(currentSetting) + 100;
        }else{
          val = getSettingValue(currentSetting) + 1;
        }

        if (inRange(val, settingRules[currentSetting][1], settingRules[currentSetting][2])) {
          setSettingValue(currentSetting, val);
          settingsLoopFlag = true;
        }
      }
    } else {
      if (currentSetting != 0) {
        currentSetting--;
        settingsLoopFlag = true;
      }
    }
  }

  // If thumbwheel is in bottom position
  else if (hallValue <= (txSettings.centerHallValue - hallMenuMargin) && settingsLoopFlag == false) {
    // Down
    if (changeSelectedSetting == true) {

      if (settingRules[currentSetting][0] != -1) {
        if(currentSetting==MINPWM || currentSetting==MAXPWM){
          val = getSettingValue(currentSetting) - 100;
        }else{
          val = getSettingValue(currentSetting) - 1;
        }

        if (inRange(val, settingRules[currentSetting][1], settingRules[currentSetting][2])) {
          setSettingValue(currentSetting, val);
          settingsLoopFlag = true;
        }
      }

    } else {
      if (currentSetting < (numOfSettings - 1)) {
        currentSetting++;
        settingsLoopFlag = true;
      }
    }
  }

  // If thumbwheel is in middle position
  else if ( inRange( hallValue, (txSettings.centerHallValue - hallMenuMargin), (txSettings.centerHallValue + hallMenuMargin) ) ) {
    settingsLoopFlag = false;
  }

  // Update selected setting to the new value
  if ( isTrigger() ) {

    if (settingsChangeFlag == false) {
      // Save settings to EEPROM
      if (changeSelectedSetting == true) {

        // Settings that need's to be transmitted to the recevier
        if ( currentSetting == TRIGGER || currentSetting == MODE ) {

          if (!transmitSetting( currentSetting, getSettingValue(currentSetting) )) {
            // Error! Load the old setting
            loadEEPROMSettings();
          }
        }

        // If new address is choosen
        else if ( currentSetting == ADDRESS ) {
          // Generate new address
          uint64_t address = generateAddress();

          if (transmitSetting( currentSetting, address )) {
            setSettingValue(currentSetting, address);
            drawTitleScreen(F("UPDATED"));
            initiateTransmitter();
          } else {
          	drawTitleScreen(F("RX ERR"));
          }
          delay(500);
        }

        // If we want to use the default address again
        else if ( currentSetting == RESET ) {

          if (transmitSetting( ADDRESS, defaultAddress )) {
            setSettingValue( ADDRESS, defaultAddress );
            drawTitleScreen(F("RESET"));
            initiateTransmitter();
          } else {
            drawTitleScreen(F("RX ERR"));
          }
          delay(500);

         // LAST EXIT SETTING MENU
        }else if ( currentSetting == numOfSettings-1) {
          if(1>=getSettingValue(currentSetting)){
              setSettingValue( currentSetting,0);
              changeSettings = false;
          }
        }

        updateEEPROMSettings();
      }

      changeSelectedSetting = !changeSelectedSetting;
      settingsChangeFlag = true;
    }

  } else {
    settingsChangeFlag = false;
  }
}

// Update values used to calculate speed and distance travelled.
void calculateRatios() {
  // Gearing ratio
  gearRatio = (float)txSettings.motorPulley / (float)txSettings.wheelPulley;
  // ERPM to Km/h
  ratioRpmSpeed = (gearRatio * 60 * (float)txSettings.wheelDiameter * 3.14156) / (((float)txSettings.motorPoles / 2) * 1000000);
  // Pulses to km travelled
  ratioPulseDistance = (gearRatio * (float)txSettings.wheelDiameter * 3.14156) / (((float)txSettings.motorPoles * 3) * 1000000);
}

// Get settings value by index (usefull when iterating through settings).
short getSettingValue(uint8_t i) {
  short v;
  switch (i) {
    case TRIGGER: v = txSettings.triggerMode;    break;
    case 1:       v = txSettings.batteryType;    break;
    case 2:       v = txSettings.batteryCells;   break;
    case 3:       v = txSettings.motorPoles;     break;
    case 4:       v = txSettings.motorPulley;    break;
    case 5:       v = txSettings.wheelPulley;    break;
    case 6:       v = txSettings.wheelDiameter;  break;
    case MODE:    v = txSettings.controlMode;    break;
    case CRUISE:  v = txSettings.stepCruise;     break;
    case 9:       v = txSettings.minHallValue;   break;
    case 10:      v = txSettings.centerHallValue;break;
    case 11:      v = txSettings.maxHallValue;  break;
    case MINPWM:  v = txSettings.minPWM;        break;
    case MAXPWM:  v = txSettings.maxPWM;        break;
    case 16:      v = txSettings.rate;          break;
    case (numOfSettings-1): v = txSettings.exit;break;
  }
  return v;
}

// Set a value of a specific setting by index.
void setSettingValue(uint8_t i, uint64_t v) {
  switch (i) {
    case TRIGGER: txSettings.triggerMode = v;     break;
    case 1:       txSettings.batteryType = v;     break;
    case 2:       txSettings.batteryCells = v;    break;
    case 3:       txSettings.motorPoles = v;      break;
    case 4:       txSettings.motorPulley = v;     break;
    case 5:       txSettings.wheelPulley = v;     break;
    case 6:       txSettings.wheelDiameter = v;   break;
    case MODE:    txSettings.controlMode = v;     break;
    case CRUISE:  txSettings.stepCruise = v;      break;
    case 9:       txSettings.minHallValue = v;    break;
    case 10:      txSettings.centerHallValue = v; break;
    case 11:      txSettings.maxHallValue = v;    break;
    case MINPWM:  txSettings.minPWM = v;          break;
    case MAXPWM:  txSettings.maxPWM = v;          break;
    case ADDRESS: txSettings.address = v;         break;
    case 16:      txSettings.rate = v;            break;
    case (numOfSettings-1): txSettings.exit = v;  break;
  }
}

// Function used to transmit the throttle value, and receive the VESC realtime.
void transmitToReceiver() {
  if ((millis() - lastTransmission) >= txSettings.rate ) {

    lastTransmission = millis();

    if (radio.write( &remPackage, sizeof(remPackage) )){
      while (radio.isAckPayloadAvailable()) {
        // Return UART datas
        radio.read( &returnData, sizeof(returnData));
      }
      // Transmission was a succes
      failCount = 0;
    } else {
      failCount++;
    }

    if (failCount < 5) {
      connected = true;
    } else {
      connected = false;
    }
  }
}


bool transmitSetting(uint8_t setting, uint64_t value){

bool flag[5]={false,false,false,false,false};
short timer=0;
bool sendRx = false;
remPackage.id=0;
setPackage.id = 1;
setPackage.setting = setting;
setPackage.value = value;
  while(timer<10000){

    if(radio.isAckPayloadAvailable()){
      if(remPackage.id==1){
        radio.read( &remPackage, sizeof(remPackage));
        if(remPackage.id==2){
          if(radio.write(&setPackage, sizeof(setPackage))){
            flag[1]=true;
          }
        }
      }else{
        radio.read( &setPackage, sizeof(setPackage));
        if(setPackage.id==2 && setPackage.setting==setting && setPackage.value==value){
          setPackage.id=3;
          if(radio.write(&setPackage, sizeof(setPackage))){
            flag[2]=true;
          }
        }else if(setPackage.id==4 && flag[2]==true){
          flag[3]=true;
          break;
        }
      }
    }

    if(remPackage.id==0){
        remPackage.type=1;
        remPackage.id=1;
        if(radio.write(&remPackage, sizeof(remPackage))){
          flag[0]=true;
        }

    }
  timer++;
}

  if(flag[0]==true && flag[1]==true && flag[2]==true && flag[3]==true){
    drawPrint("RX Accepted");
    delay(3000);
    return true;
  }else{
    drawPrint("RX Rejected");
    delay(3000);
  }
  return false;
}


void initiateTransmitter() {
  radio.begin();
  radio.setChannel(defaultChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(txSettings.address);
  radio.setRetries(15, 15);
}


void updateMainDisplay()
{
  u8g2.firstPage();

  do {
    if ( changeSettings == true ){
      drawSettingsMenu();
    }else{
      drawThrottle();
      drawPage();
      drawBatteryLevel();
      drawSignal();
    }
  } while ( u8g2.nextPage() );
}

// 0-255
void getThrottlePosition(){
  calculateThrottlePosition(calculateHallSensorPosition());
}
// Get average reading from HALL SENSOR
short calculateHallSensorPosition(){
  unsigned short total = 0;
  for ( uint8_t i = 0; i < 10; i++ )
  {
    total += analogRead(hallSensorPin);
  }

  return (hallValue = total / 10);
}
// Output analog 0-255
void calculateThrottlePosition(short _input){
  if ( _input >= txSettings.centerHallValue ){
    throttle = constrain( map(_input, txSettings.centerHallValue, txSettings.maxHallValue, 1500, 2300), hallCenterNoise, txSettings.maxPWM );
  }else{
    throttle = constrain( map(_input, txSettings.minHallValue, txSettings.centerHallValue, 700, 1500), txSettings.minPWM, hallCenterNoise );
  }
  if (abs(throttle - hallCenterNoise) < hallCenterMargin ){ throttle = hallCenterNoise; }

}


uint8_t batteryLevel() {
  float voltage = batteryVoltage();

  if (voltage <= minVoltage) {
    return 0;
  } else if (voltage >= maxVoltage) {
    return 100;
  } else {
    return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
  }
}
float batteryVoltage()
{
  float batteryVoltage = 0.0;
  unsigned short total = 0;

  for (uint8_t i = 0; i < 10; i++) {
    total += analogRead(batteryMeasurePin);
  }

  batteryVoltage = (refVoltage / 1024.0) * ((float)total / 10.0);

  return batteryVoltage;
}

// To save precious SRAM we define function to get the setting titles
String getSettingTitle(uint8_t index) {
  String title;

  switch (index) {
    case 0: title = "Trigger use";    break;
    case 1: title = "Battery type";   break;
    case 2: title = "Battery cells";  break;
    case 3: title = "Motor poles";    break;
    case 4: title = "Motor pulley";   break;
    case 5: title = "Wheel pulley";   break;
    case 6: title = "Wheel diameter";   break;
    case 7: title = "Control mode";     break;
    case CRUISE: title = "Cruise Stepper";      break;
    case 9: title = "Throttle min";     break;
    case 10: title = "Throttle center";  break;
    case 11: title = "Throttle max";      break;
    case MINPWM: title = "Minimum PWM";          break;
    case MAXPWM: title = "Maximum PWM";         break;
    case ADDRESS: title = "Generate address";  break;
    case RESET: title = "Reset address";     break;
    case 16: title = "Rate TX - RX";     break;
    case 17: title = "EXIT SETTING";     break;
  }

  return title;
}

void drawSettingsMenu() {
  // To save SRAM we define strings local only to be used then changing settings
  String settingValues[4][3] = {
    {"Safety ON", "Safety OFF", "Cruiser"},
    {"Li-ion", "LiPo", ""},
    {"PPM", "PPM and UART", "UART"},
    {"NO", "YES", ""}
  };

  String settingUnits[3] = {"S", "T", "mm"};
  String settingValue, settingUnit;

  // Used to map setting values and units to the right setting number
  uint8_t unitIdentifier[numOfSettings]  = {0, 0, 1, 0, 2, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t valueIdentifier[numOfSettings] = {1, 2, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};

  // Base coordinates
  uint8_t x = 0;
  uint8_t y = 10;

  // Local variable to store the setting value
  uint64_t value;

  // Print setting title
  tempString = getSettingTitle( currentSetting );
  drawString(tempString, tempString.length() + 1, x, y, u8g2_font_profont12_tr );

  // Get current setting value

  switch (currentSetting) {
    case ADDRESS:
      value = txSettings.address;
      break;
    case RESET:
      value = defaultAddress;
      break;
    default:
      value = getSettingValue(currentSetting);
      break;
  }

  if ( valueIdentifier[currentSetting] != 0 ){
    uint8_t index = valueIdentifier[ currentSetting ] - 1;
    settingValue = settingValues[ index ][ value ];
  }else{
    if (currentSetting == ADDRESS || currentSetting == RESET) {
      settingValue = uint64ToAddress(value);
    } else {
      settingValue = uint64ToString(value);
    }
  }

  if ( unitIdentifier[ currentSetting ] != 0 ) {
    settingUnit = settingUnits[ unitIdentifier[ currentSetting ] - 1 ];
  }

  tempString = settingValue + settingUnit;
  if ( changeSelectedSetting == true ){
    drawString(tempString, tempString.length() + 1, x + 10, y + 20, u8g2_font_10x20_tr );

    if ( inRange(currentSetting, 9, 11) ) {
      tempString = "(" + (String)hallValue + ")";
      drawString(tempString, 8, x + 92, y + 20, u8g2_font_profont12_tr );
    }
  }else{
    drawString(tempString, tempString.length() + 1, x, y + 20, u8g2_font_10x20_tr );
  }
}


void drawStartScreen() {
  u8g2.firstPage();

  do {

    u8g2.drawXBM( 4, 4, 24, 24, logo);

    tempString = F("DIY Controller");
    drawString(tempString, 15, 33, 21, u8g2_font_helvR10_tr );

  } while ( u8g2.nextPage() );

  delay(1500);
}

void drawTitleScreen(String title) {
  u8g2.firstPage();

  do {

    drawString(title, 20, 12, 20, u8g2_font_helvR10_tr );

  } while ( u8g2.nextPage() );

  delay(1500);
}

void drawPrint(String title) {
  u8g2.firstPage();
  do {

    drawString(title, 20, 12, 20, u8g2_font_helvR10_tr );

  } while ( u8g2.nextPage() );
}

void drawPage() {

  float value;
  uint8_t decimals;
  String suffix;

  short first, last;

  uint8_t x = 0;
  uint8_t y = 16;

  value = ratioRpmSpeed * returnData.rpm;
  suffix = F("K/h");
  decimals = 2;

  // Display Voltage
  tempString = (returnData.inpVoltage) + (String)"v";
  drawString(tempString, 10, x + 60, y - 10, u8g2_font_synchronizer_nbp_tr);

  // Amps HOUR
  tempString = (returnData.ampHours) + (String)"amp";
  drawString(tempString, 12, x, y - 10, u8g2_font_synchronizer_nbp_tr);

  // Total Distant KM
  tempString = (String)(ratioPulseDistance * returnData.tachometerAbs) + "Km";
  drawString(tempString, 12, x, y , u8g2_font_synchronizer_nbp_tr);

  // Split up the float value: a number, b decimals.
  first = abs( floor(value) );
  last = value * pow(10, 3) - first * pow(10, 3);

  // Add leading zero
  if ( first <= 9 ) {
    tempString = "0" + (String)first;
  } else {
    tempString = (String)first;
  }

  // Display numbers :: SPEED
  drawString(tempString, 10, x + 60, y + 16, u8g2_font_logisoso18_tn );

  // Display decimals
  // Add leading zero
  if ( last <= 9 ) {
    tempString = "0" + (String)last;
  } else {
    tempString = (String)last;
  }
  tempString = "." + (String)tempString;
  drawString(tempString, decimals + 2, x + 86, y + 6, u8g2_font_synchronizer_nbp_tr);

  // Display suffix
  drawString(suffix, 10, x + 88, y + 14, u8g2_font_synchronizer_nbp_tr);

  // Display MODE
  switch (txSettings.triggerMode) {
    case 1:
      tempString = "SAFETY OFF";
      break;

    case 2:
      tempString = "CRUISE";
      break;

    default:
      tempString = "SAFETY ON";
      break;
  }
  if (txSettings.triggerMode == 2) {
    if (CruiseActivated) {
      tempString = tempString + " ON";
    } else {
      tempString = tempString + " OFF";
    }
  }
  drawString(tempString, 12, x + 2, y + 15, u8g2_font_profont10_tf );
}

void drawString(String text, uint8_t lenght, uint8_t x, uint8_t y, const uint8_t  *font) {

  static char textBuffer[20];

  text.toCharArray(textBuffer, lenght);

  u8g2.setFont(font);
  u8g2.drawStr(x, y, textBuffer);
}

void drawThrottle() {
  uint8_t x = 0;
  uint8_t y = 18;
  uint8_t width;

  // Draw throttle
  u8g2.drawHLine(x, y, 52);
  u8g2.drawVLine(x, y, 5);
  u8g2.drawVLine(x + 52, y, 5);
  u8g2.drawHLine(x, y + 5, 5);
  u8g2.drawHLine(x + 52 - 4, y + 5, 5);

  if (throttle >= hallCenterNoise) {
    width = map(throttle, hallCenterNoise, txSettings.maxPWM, 0, 49);

    for (uint8_t i = 0; i < width; i++)
    {
      u8g2.drawVLine(x + i + 2, y + 2, 3);
    }
  } else {
    width = map(throttle, txSettings.minPWM, hallCenterNoise-1, 49, 0);

    for (uint8_t i = 0; i < width; i++)
    {
      u8g2.drawVLine(x + 50 - i, y + 2, 3);
    }
  }
}

void drawSignal() {
  // Position on OLED
  uint8_t x = 114;
  uint8_t y = 17;

  if (connected == true) {
    if (isTrigger()) {
      u8g2.drawXBM(x, y, 12, 12, transmittingIcon);
    } else {
      u8g2.drawXBM(x, y, 12, 12, connectedIcon);
    }
  } else {
    if (millis() - lastSignalBlink > 500) {
      signalBlink = !signalBlink;
      lastSignalBlink = millis();
    }

    if (signalBlink == true) {
      u8g2.drawXBM(x, y, 12, 12, connectedIcon);
    } else {
      u8g2.drawXBM(x, y, 12, 12, noconnectionIcon);
    }
  }
}

void drawBatteryLevel() {
  // Position on OLED
  uint8_t x = 108;
  uint8_t y = 4;

  uint8_t level = batteryLevel();

  u8g2.drawFrame(x + 2, y - 4, 18, 9);
  u8g2.drawBox(x, y - 2, 2, 5);

  for (uint8_t i = 0; i < 5; i++) {
    uint8_t p = round((100 / 5) * i);
    if (p <= level)
    {
      u8g2.drawBox(x + 4 + (3 * i), y - 2, 2, 5);
    }
  }
}


uint64_t generateAddress()
{
  randomSeed( millis() );

  char x[10];
  const char *hex = "0123456789ABCDEF";
  const char *str = "12346789BCDE";

  for (uint8_t i = 0 ; i < 10; i++ )
  {
    char y;

    if (i == 0) y = str[ random(0, 12) ];
    else if (i == 1) y = str[ random(0, 12) ];
    else y = hex[ random(0, 16) ];

    x[i] = y;
  }

  // Convert hex char array to uint64_t
  return (uint64_t) StringToUint64(x);
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

// Convert hex String to uint64_t: http://forum.arduino.cc/index.php?topic=233813.0
uint64_t StringToUint64( char * string ) {
  uint64_t x = 0;
  char c;

  do {
    c = hexCharToBin( *string++ );
    if (c < 0)
      break;
    x = (x << 4) | c;
  } while (1);

  return x;
}

char hexCharToBin(char c) {
  if (isdigit(c)) {  // 0 - 9
    return c - '0';
  } else if (isxdigit(c)) { // A-F, a-f
    return (c & 0xF) + 9;
  }
  return -1;
}


void setDefaultEEPROMSettings() {
  clearEEPROM();

  for ( uint8_t i = 0; i < numOfSettings; i++ )
  {
    setSettingValue( i, settingRules[i][0] );
  }

  txSettings.address = defaultAddress;
  updateEEPROMSettings();
}
void loadEEPROMSettings() {
  // Load settings from EEPROM to custom struct
  EEPROM.get(0, txSettings);

  bool rewriteSettings = false;

  // Loop through all settings to check if everything is fine
  for (uint8_t i = 0; i < numOfSettings; i++) {

    // If setting default value is -1, don't check if its valid
    if ( settingRules[i][0] != -1 ) {

      short val = getSettingValue(i);

      if (! inRange(val, settingRules[i][1], settingRules[i][2])) {
        // Setting is damaged or never written. Rewrite default.
        rewriteSettings = true;
        setSettingValue(i, settingRules[i][0] );
      }
    }
  }

  if (rewriteSettings == true) {
    updateEEPROMSettings();
  } else {
    // Calculate constants
    calculateRatios();
  }
}

void updateEEPROMSettings() { EEPROM.put(0, txSettings); calculateRatios(); }
void clearEEPROM() { for (int i = 0 ; i < EEPROM.length() ; i++){ EEPROM.write(i, 0); } }

void cruise_stepper(){
	if(CruiseValue > (txSettings.centerHallValue)){ CruiseValue-=txSettings.stepCruise;	}
  else if(CruiseValue < (txSettings.centerHallValue)){ CruiseValue+=txSettings.stepCruise; }
  calculateThrottlePosition(CruiseValue);
}
void cruise_mode(){
  if (isTrigger()){
    if (!isNeutral()){

      if(!CruiseActivated){ CruiseValue=hallValue; }

          if(hallValue > (txSettings.centerHallValue) && hallValue < (txSettings.maxHallValue) && hallValue > CruiseValue){
              CruiseValue+=txSettings.stepCruise;
          }else if(hallValue < (txSettings.centerHallValue) && hallValue > (txSettings.minHallValue) && hallValue < CruiseValue){
              CruiseValue-=txSettings.stepCruise;
          }

                CruiseActivated = CruiseSafety = true;

    }else{
        CruiseActivated=true;
        if(!CruiseSafety){ CruiseValue=hallValue; }
    }

        calculateThrottlePosition(CruiseValue);

  }else{

    if(isNeutral()){ CruiseActivated=false; }

    if(CruiseActivated){ cruise_stepper();
    }else{
      if(!isNeutral()){ CruiseSafety=false; }
      if(CruiseSafety){ cruise_stepper(); }
    }
  }

  remPackage.throttle = throttle;
}

bool inRange(short v, short min, short max) { return ((min <= v) && (v <= max)); }
bool isTrigger() { return (digitalRead(triggerPin) == LOW); }
bool isNeutral(){ return (throttle==hallCenterNoise); }
void setNeutral(){calculateThrottlePosition(txSettings.centerHallValue);remPackage.throttle = throttle;}
