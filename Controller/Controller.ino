#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RF24.h>
#include <VescUart.h>

#define PROFILE 0
#define MODE 1
#define BATTERY_TYPE 2
#define BATTERY_CELLS 3
#define MOTOR_POLES 4
#define MOTOR_PULLEY 5
#define WHEEL_PULLEY 6
#define WHEEL_DIAMETER 7
#define STEPPER 8
#define ADDRESS 15
#define RESET 16
#define RATE 17

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const char settingValues[4][4][10] = {
  {"PROFILE-A","PROFILE-B","PROFILE-C"},

  {"SAFETY", "NORMAL", "Cruiser", "Beginner"},
  {"Li-ion", "LiPo"},
  {"NO", "YES"},
};

const unsigned char logo[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01,
  0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19,
  0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37,
  0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01,
  0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07,
  0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char transmittingIcon[] PROGMEM = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
  0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char connectedIcon[] PROGMEM = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
  0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char noconnectionIcon[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

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
  short Kp; //0.0001* // 0.0075
  short Ki; //0.00001* // 0.00065
  short Kd; //0.0001* // 0.0015
  short ratePing;
};
struct settingPackage {
  uint8_t id;
  uint8_t setting;
  uint64_t value;
};
struct settings {
  uint8_t profileMode;
  uint8_t controlMode;
  uint8_t batteryType;
  uint8_t batteryCells;
  uint8_t motorPoles;
  uint8_t motorPulley;
  uint8_t wheelPulley;
  uint8_t wheelDiameter;
  short minHallValue;
  short centerHallValue;
  short maxHallValue;
   uint8_t stepper;
   short Kp;
   short Ki;
   short Kd;
  uint64_t address;
  uint8_t rate;
  uint8_t exit;

  uint8_t A_controlMode;
  uint8_t A_batteryType;
  uint8_t A_batteryCells;
  uint8_t A_motorPoles;
  uint8_t A_motorPulley;
  uint8_t A_wheelPulley;
  uint8_t A_wheelDiameter;
  uint8_t A_stepper;
  uint64_t A_address;

  uint8_t B_controlMode;
  uint8_t B_batteryType;
  uint8_t B_batteryCells;
  uint8_t B_motorPoles;
  uint8_t B_motorPulley;
  uint8_t B_wheelPulley;
  uint8_t B_wheelDiameter;
  uint8_t B_stepper;
  uint64_t B_address;
};

uint8_t currentSetting = 0;
const uint8_t numOfSettings = 19;

const short settingRules[numOfSettings][3] {
  {0, 0, 3}, // PROFILE
  {1, 0, 3}, // 0:ECO 1:NORMAL 2:CRUISE 3:BEGINNER
  {0, 0, 1}, // 0 Li-ion & 1 LiPo
  {10, 1, 14},
  {14, 1, 250},
  {14, 1, 250},
  {66, 1, 250},
  {100, 0, 250},
  { 10, 1, 100}, // 8 STEPPER SPEED
  { 75, 0, 1000}, // 9 STEPPER Kp  0.0001*
  { 65, 0, 1000}, // 10 STEPPER Ki 0.00001*
  { 15, 0, 1000}, // 11 STEPPER Kd 0.0001*
  {0, 0, 1023}, // 12
  {500, 0, 1023}, // 13
  {1000, 0, 1023}, // 14
  { -1, 0, 0}, // 17 No validation for pipe address (not really possible this way)
  { -1, 0, 0}, // 18 No validation for default address
  { 50, 1, 100}, // 19 Timing TX rate by millionsecond
  { 0, 0, 1} // 20 EXIT
};

struct packageRX returnData;
struct packageTX txPacket;
struct settingPackage setPackage;
struct settings txSettings;


float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

const uint8_t triggerPin = 4;
const uint8_t batteryMeasurePin = A2;
const uint8_t hallSensorPin = A3;

float percentVoltage = 0;
const float minVoltage = 3.0;
const float maxVoltage = 4.2;
const float refVoltage = 5.0;

unsigned short hallValue, throttle, throttle_set, speedKM;
const uint8_t hallCenterMargin = 4;
const uint8_t hallMenuMargin = 100;
const short hallCenterNoise = 1500;

const uint64_t defaultAddress = 0xE8E8F0F1E9LL;
const uint8_t defaultChannel = 108;
unsigned long lastTransmission, lastPing;
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

RF24 radio(9, 10);
bool safety[3] = {false,false,false};

void setup() {
  //setDefaultEEPROMSettings();
  loadEEPROMSettings();

  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
  pinMode(batteryMeasurePin, INPUT);

  u8g2.begin();
  drawStartScreen();

  getThrottlePosition();
  if(isTrigger()){
    drawTitleScreen(F("Remote Settings"));
    changeSettings=true;
  }
  //changeSettings=true;

  initiateTransmitter();
}

uint8_t it = 0;
unsigned long timeTrigger;
bool flagTrigger[3] = {false,false,false};

void loop() {
  getThrottlePosition();
  // Ping delay time to RX
  // Ping refresh time to UART
  txPacket.ratePing=100;

  if (changeSettings == true) {
    txPacket.type=1;
    txPacket.id=0;

    if((millis() - lastPing)>txPacket.ratePing){
      lastPing=millis();
    if(radio.write(&txPacket, sizeof(txPacket))){
      if(radio.isAckPayloadAvailable()){
        radio.read(&txPacket, sizeof(txPacket));
      }
    }}

    controlSettingsMenu();
  }else{
    txPacket.type = 0;
    txPacket.trigger = isTrigger();

    // Throttle is at Neutral position
    if (!safety[0] && throttle==hallCenterNoise) { safety[0] = true; }

    // CRUISE CONTROL MODE x2 Trigger
    if(txSettings.controlMode==2){
      bool nThrottle = inRange(hallValue, (txSettings.centerHallValue-getTxValue(STEPPER)), (txSettings.centerHallValue+getTxValue(STEPPER)));
      if(txPacket.trigger){
        timeTrigger=millis();
        flagTrigger[0]=flagTrigger[1]=true;
        if(!nThrottle){ flagTrigger[2]=true; }else{ flagTrigger[2]=false; }
      }

      if(flagTrigger[0] || flagTrigger[1]){
        unsigned short secTrigger = (millis()-timeTrigger);
        if(secTrigger>500 && flagTrigger[1]){
          it=0; flagTrigger[0] = flagTrigger[0] = false;
        }else if(flagTrigger[0] && secTrigger>50){
          it++; flagTrigger[0] = false;
        }
      }
      if(it>2 || !connected || (!nThrottle && !flagTrigger[2])){
        it=0;
      }else if(it==2){
        txPacket.trigger=true;
        flagTrigger[1] = false;
      }
      if(returnData.cruise==true && txPacket.trigger && nThrottle){
        throttle = returnData.throttle_set;
      }
        txPacket.throttle = throttle;
    }else{
          // controlMode
          // 0 = S-ON
          // 1 = S-OFF
          // 2 = CRUISE
          // 3 = BEGINNER 15km limit

          // 0 = SAFETY ON
          if(txSettings.controlMode==0){
            if(throttle>hallCenterNoise && !txPacket.trigger){
              safety[1]=false;
            }else if(throttle<=hallCenterNoise && txPacket.trigger){
              safety[1]=true;
            }

            if(txPacket.trigger && safety[1]){
              txPacket.throttle = throttle;
            }else{
              setNeutral();
            }
            /*
            if(throttle>hallCenterNoise){
              if(throttle_set<hallCenterNoise){ throttle_set=hallCenterNoise; }
              if((throttle_set+getTxValue(STEPPER))>throttle && throttle_set<throttle){
                throttle=throttle_set++;
              }else{
                if(throttle>throttle_set && throttle_set<throttle){
                  throttle=throttle_set+=getTxValue(STEPPER);
                }else{
                  throttle_set=throttle;
                }
              }
            }else{
              throttle_set=throttle;
            }
            txPacket.throttle = throttle_set;
            //*/

          // 3 = BEGINNER 20km limit
          }else if(txSettings.controlMode==3){

            unsigned short speedLimit = 20;
            speedKM = ratioRpmSpeed * returnData.rpm;

            if(throttle>hallCenterNoise){
              if(throttle_set<hallCenterNoise){ throttle_set=hallCenterNoise; }

              if(speedKM>speedLimit){ throttle=throttle_set-=getTxValue(STEPPER);
              }else if((throttle_set+getTxValue(STEPPER))>throttle && throttle_set<throttle){
                throttle=throttle_set++;
              }else{
                if(throttle>throttle_set && throttle_set<throttle){
                  throttle=throttle_set+=getTxValue(STEPPER);
                }else{
                  throttle_set=throttle;
                }
              }

            }else if(throttle<hallCenterNoise){
              if(throttle>throttle_set){ throttle_set=throttle; }
              if(throttle<1200){
                throttle=throttle_set-=25;
              }else if(throttle<throttle_set){
                throttle=throttle_set-=getTxValue(STEPPER);
              }
            }else{
              if(throttle<throttle_set){ throttle_set=throttle; }
              throttle=throttle_set;
            }
            txPacket.throttle = throttle_set;
          }else{
            // S-OFF MODE
            txPacket.throttle = throttle;
          }

        }

    if(!safety[0]){ setNeutral(); }
    transmitToReceiver();
  }

  // Call function to update display
  updateMainDisplay();
}

// When called the throttle and trigger will be used to navigate and change settings
void controlSettingsMenu() {
  short v;
  // If thumbwheel is in top position
  if (hallValue >= (txSettings.centerHallValue + hallMenuMargin) && settingsLoopFlag == false) {
    // Up
    if (changeSelectedSetting == true) {
      if (settingRules[currentSetting][0] != -1) {
          v = getTxValue(currentSetting) + 1;
        if (inRange(v, settingRules[currentSetting][1], settingRules[currentSetting][2])) {
          setTxValue(currentSetting, v);
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
          v = getTxValue(currentSetting) - 1;
        if (inRange(v, settingRules[currentSetting][1], settingRules[currentSetting][2])) {
          setTxValue(currentSetting, v);
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
        if ( currentSetting == MODE || currentSetting == STEPPER) {

          if (!transmitSetting( currentSetting, getTxValue(currentSetting))) {
            // Error! Load the old setting
            loadEEPROMSettings();
          }
        }

        // If new address is choosen
        else if ( currentSetting == ADDRESS ) {
          // Generate new address
          uint64_t address = generateAddress();

          if (transmitSetting( currentSetting, address )) {
            setTxValue(currentSetting, address);
            initiateTransmitter();

          }
        }

        // If we want to use the default address again
        else if ( currentSetting == RESET ) {

          if (transmitSetting( ADDRESS, defaultAddress )) {
            setTxValue( ADDRESS, defaultAddress );
            initiateTransmitter();
          }

         // LAST EXIT SETTING MENU
        }else if ( currentSetting == numOfSettings-1) {
          if(getTxValue(currentSetting)>0){
              setTxValue( currentSetting,0);
              initiateTransmitter();
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
  gearRatio = (float)getTxValue(MOTOR_PULLEY) / (float)getTxValue(WHEEL_PULLEY);
  // ERPM to Km/h
  //ratioRpmSpeed = ((float)getTxValue(WHEEL_DIAMETER) * 3.14156) / ((float)getTxValue(MOTOR_POLES) / 2) * (0.00003728226 * gearRatio);
  ratioRpmSpeed = ((gearRatio * 60 * (float)getTxValue(WHEEL_DIAMETER) * 3.14156)) / ((float)getTxValue(MOTOR_POLES) / 2) * (1000000);

  // Pulses to km travelled
  ratioPulseDistance = (gearRatio*(float)getTxValue(WHEEL_DIAMETER) * 3.14156) / (((float)getTxValue(MOTOR_POLES) / 2) * 1000000);
}

// Get settings value by index (usefull when iterating through settings).
short getTxValue(uint8_t i) {
  short v;

  switch (i) {
    case PROFILE:    v = txSettings.profileMode;    break;
    case MODE:
        switch (txSettings.profileMode){
          case 1:    v = txSettings.A_controlMode;    break;
          case 2:    v = txSettings.B_controlMode;    break;
          default: v = txSettings.controlMode;
        }
    break;
    case BATTERY_TYPE:
          switch (txSettings.profileMode){
            case 1:    v = txSettings.A_batteryType;    break;
            case 2:    v = txSettings.B_batteryType;    break;
            default: v = txSettings.batteryType;
          }
    break;
    case BATTERY_CELLS:
            switch (txSettings.profileMode){
              case 1:    v = txSettings.A_batteryCells;    break;
              case 2:    v = txSettings.B_batteryCells;    break;
              default: v = txSettings.batteryCells;
            }
    break;
    case MOTOR_POLES:
            switch (txSettings.profileMode){
              case 1:    v = txSettings.A_motorPoles;    break;
              case 2:    v = txSettings.B_motorPoles;    break;
              default: v = txSettings.motorPoles;
            }
   break;
   case MOTOR_PULLEY:
              switch (txSettings.profileMode){
                case 1:    v = txSettings.A_motorPulley;    break;
                case 2:    v = txSettings.B_motorPulley;    break;
                default: v = txSettings.motorPulley;
              }
   break;
   case WHEEL_PULLEY:
                switch (txSettings.profileMode){
                  case 1:    v = txSettings.A_wheelPulley;    break;
                  case 2:    v = txSettings.B_wheelPulley;    break;
                  default: v = txSettings.wheelPulley;
                }
    break;
    case WHEEL_DIAMETER:
                  switch (txSettings.profileMode){
                    case 1:    v = txSettings.A_wheelDiameter;    break;
                    case 2:    v = txSettings.B_wheelDiameter;    break;
                    default: v = txSettings.wheelDiameter;
                  }
    break;
    case STEPPER:
                  switch (txSettings.profileMode){
                    case 1:    v = txSettings.A_stepper;    break;
                    case 2:    v = txSettings.B_stepper;    break;
                    default: v = txSettings.stepper;
                  }
    break;
    case 9:   txPacket.Kp = v = txSettings.Kp;    break;
    case 10:  txPacket.Ki = v = txSettings.Ki;    break;
    case 11:  txPacket.Kd = v = txSettings.Kd;    break;
    case 12:       v = txSettings.minHallValue;   break;
    case 13:      v = txSettings.centerHallValue; break;
    case 14:      v = txSettings.maxHallValue;   break;
    case (numOfSettings-2): v = txSettings.rate; break;
    case (numOfSettings-1): v = txSettings.exit; break;
  }
  return v;
}

// Set a value of a specific setting by index.
void setTxValue(uint8_t i, uint64_t v) {
  switch (i) {
    case PROFILE:    txSettings.profileMode = v;     break;
    case MODE:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_controlMode = v;    break;
        case 2:    txSettings.B_controlMode = v;    break;
        default: txSettings.controlMode = v;
      }
    break;
    case BATTERY_TYPE:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_batteryType = v;    break;
        case 2:    txSettings.B_batteryType = v;    break;
        default: txSettings.batteryType = v;
      }
    break;
    case BATTERY_CELLS:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_batteryCells = v;    break;
        case 2:    txSettings.B_batteryCells = v;    break;
        default: txSettings.batteryCells = v;
      }
    break;
    case MOTOR_POLES:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_motorPoles = v;    break;
        case 2:    txSettings.B_motorPoles = v;    break;
        default: txSettings.motorPoles = v;
      }
    break;
    case MOTOR_PULLEY:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_motorPulley = v;    break;
        case 2:    txSettings.B_motorPulley = v;    break;
        default: txSettings.motorPulley = v;
      }
    break;
    case WHEEL_PULLEY:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_wheelPulley = v;    break;
        case 2:    txSettings.B_wheelPulley = v;    break;
        default: txSettings.wheelPulley = v;
      }
    break;
    case WHEEL_DIAMETER:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_wheelDiameter = v;    break;
        case 2:    txSettings.B_wheelDiameter = v;    break;
        default: txSettings.wheelDiameter = v;
      }
    break;
    case STEPPER:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_stepper = v;    break;
        case 2:    txSettings.B_stepper = v;    break;
        default: txSettings.stepper = v;
      }
    break;
    case 9:       txSettings.Kp = v;      break;
    case 10:      txSettings.Ki = v;      break;
    case 11:      txSettings.Kd = v;      break;
    case 12:      txSettings.minHallValue = v;    break;
    case 13:      txSettings.centerHallValue = v; break;
    case 14:      txSettings.maxHallValue = v;    break;
    case ADDRESS:
      switch (txSettings.profileMode){
        case 1:    txSettings.A_address = v;    break;
        case 2:    txSettings.B_address = v;    break;
        default: txSettings.address = v;
      }
    break;
    case (numOfSettings-2): txSettings.rate = v;  break;
    case (numOfSettings-1): txSettings.exit = v;  break;
  }
}

// Function used to transmit the throttle value, and receive the VESC realtime.
void transmitToReceiver() {

  if ((millis() - lastTransmission) >=getTxValue(RATE)) {

    lastTransmission = millis();

    if (radio.write( &txPacket, sizeof(txPacket) )){
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

  bool flag[3]={false,false,false};
  unsigned short timer=0;
  txPacket.id=0;
  while(timer<1000){
      if(radio.isAckPayloadAvailable()){
        if(txPacket.id==1){
          radio.read( &txPacket, sizeof(txPacket));
          if(txPacket.id==2){
            setPackage.id = 1;
            setPackage.setting = setting;
            setPackage.value = value;
            if(radio.write(&setPackage, sizeof(setPackage))){
              delay(15);
              flag[1]=true;
            }
          }
        }else{
          radio.read( &setPackage, sizeof(setPackage));
          if(setPackage.id==2){
            flag[2]=true;
            break;
          }

        }

      }

      if(txPacket.id==0){
          txPacket.type=1;
          txPacket.id=1;
          if(radio.write(&txPacket, sizeof(txPacket))){
            delay(15);
            flag[0]=true;
          }

      }
    timer++;
  }
    if(flag[0]==true && flag[1]==true && flag[2]==true){
      drawPrint("RX Accepted");
      delay(3000);
      return true;
    }else{
      drawPrint("No responding.");
      delay(1500);
    }
    return false;
}


void initiateTransmitter() {
  uint64_t rx_address;
  switch(txSettings.profileMode){
    case 1:    rx_address = txSettings.A_address;    break;
    case 2:    rx_address = txSettings.B_address;    break;
    default:   rx_address =  txSettings.address;
  }
  radio.begin();
  radio.setChannel(defaultChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(rx_address);
  radio.setRetries(15, 15);
}


void updateMainDisplay(){
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
    throttle = constrain( map(_input, txSettings.centerHallValue, txSettings.maxHallValue, 1500, 2000), hallCenterNoise, 2000);
  }else{
    throttle = constrain( map(_input, txSettings.minHallValue, txSettings.centerHallValue, 1000, 1500), 1000, hallCenterNoise );
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

float batteryVoltage(){
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
    case PROFILE: title = "Control Profile";     break;
    case MODE: title = "Control Mode";    break;
    case 2: title = "Battery type";   break;
    case 3: title = "Battery cells";  break;
    case 4: title = "Motor poles";    break;
    case 5: title = "Motor pulley";   break;
    case 6: title = "Wheel pulley";   break;
    case 7: title = "Wheel diameter";   break;
    case STEPPER: title = "Acceleration";      break;
    case 9: title = "Cruise PID Kp";      break;
    case 10: title = "Cruise PID Ki";      break;
    case 11: title = "Cruise PID kd";      break;
    case 12: title = "Throttle min";     break;
    case 13: title = "Throttle center";  break;
    case 14: title = "Throttle max";      break;
    case ADDRESS: title = "Generate address";  break;
    case RESET: title = "Reset address";     break;
    case (numOfSettings-2): title = "Transmit RATE ms";     break;
    case (numOfSettings-1): title = "EXIT SETTING";     break;
  }

  return title;
}

void drawSettingsMenu() {
  String settingUnits[3] = {"S", "T", "mm"};
  String settingValue, settingUnit;

  // Used to map setting values and units to the right setting number
  uint8_t unitIdentifier[numOfSettings]  = {0,0,0,1,0,2,2,3,0,0,0,0,0,0,0,0,0,0,0};
  uint8_t valueIdentifier[numOfSettings] = {1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4};

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
      switch (txSettings.profileMode){
        case 1:    value = txSettings.A_address;    break;
        case 2:    value = txSettings.B_address;    break;
        default: value = txSettings.address;
      }
      break;
    case RESET:
      value = defaultAddress;
      break;
    default:
      value = getTxValue(currentSetting);
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

    if ( inRange(currentSetting, 12, 14) ) {
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

    u8g2.drawXBMP( 4, 4, 24, 24, logo);

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

  // Display Voltage in percent
  percentVoltage = (((returnData.inpVoltage/getTxValue(BATTERY_CELLS))-minVoltage)*(100/(maxVoltage-minVoltage)));
  if(percentVoltage>125 || percentVoltage<0) percentVoltage = 0;
  drawString("%", 5, x + 58, y - 10, u8g2_font_synchronizer_nbp_tr);
  drawString((String)percentVoltage, 10, x + 64, y - 5, u8g2_font_helvR10_tr);

  // Amps HOUR
  tempString = (returnData.ampHours)+(String)"amp";
  drawString(tempString, 12, x, y - 10, u8g2_font_synchronizer_nbp_tr);

  // Total Distant KM
  tempString = (ratioPulseDistance * returnData.tachometerAbs)+(String)"Km";
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
  drawString(tempString, 10, x + 65, y + 12, u8g2_font_helvR10_tr );

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
  switch (txSettings.controlMode) {
    case 0: tempString = "SAFETY"+(String)((safety[1]) ? " ON" : " OFF"); break;
    case 1: tempString = "NORMAL"; break;
    case 2:
      tempString = "CRUISE"+(String)((returnData.cruise) ? " ON" : " OFF");

    break;
    case 3: tempString = "BEGINNER"; break;
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
    width = map(throttle, hallCenterNoise, 2000, 0, 49);

    for (uint8_t i = 0; i < width; i++)
    {
      u8g2.drawVLine(x + i + 2, y + 2, 3);
    }
  } else {
    width = map(throttle, 1000, hallCenterNoise-1, 49, 0);

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
      u8g2.drawXBMP(x, y, 12, 12, transmittingIcon);
    } else {
      u8g2.drawXBMP(x, y, 12, 12, connectedIcon);
    }
  } else {
    if (millis() - lastSignalBlink > 100) {
      signalBlink = !signalBlink;
      lastSignalBlink = millis();
    }

    if (signalBlink == true) {
      u8g2.drawXBMP(x, y, 12, 12, connectedIcon);
    } else {
      u8g2.drawXBMP(x, y, 12, 12, noconnectionIcon);
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


uint64_t generateAddress(){
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

String uint64ToString(uint64_t number){
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  if (part1 == 0) {
    return String(part2, DEC);
  }
  return String(part1, DEC) + String(part2, DEC);
}

String uint64ToAddress(uint64_t number){
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
    setTxValue( i, settingRules[i][0] );
  }

  txSettings.address=txSettings.A_address=txSettings.B_address=defaultAddress;
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

      if (! inRange(getTxValue(i), settingRules[i][1], settingRules[i][2])) {
        // Setting is damaged or never written. Rewrite default.
        changeSettings = rewriteSettings = true;
        setTxValue(i, settingRules[i][0] );
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

bool inRange(short v, short min, short max) { return ((min <= v) && (v <= max)); }
bool isTrigger() { return (digitalRead(triggerPin) == LOW); }
void setNeutral(){calculateThrottlePosition(txSettings.centerHallValue); txPacket.throttle = throttle; }
