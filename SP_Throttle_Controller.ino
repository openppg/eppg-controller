#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
//#include <Adafruit_ZeroDMA.h>
#include <Servo.h> // to control ESCs
#include <Adafruit_BMP3XX.h>
#include <Adafruit_DRV2605.h>
#include <extEEPROM.h>
#include <TimeLib.h>

#define BUZ_PIN 5
#define BTN_PIN 6
#define POT_PIN A0
#define TFT_RST 9
#define TFT_CS 10
#define TFT_DC 11
#define TFT_LITE A1
#define ESC_PIN 12

#define BLACK                 ST77XX_BLACK
#define WHITE                 ST77XX_WHITE
#define GREEN                 ST77XX_GREEN
#define YELLOW                ST77XX_YELLOW
#define RED                   ST77XX_RED
#define BLUE                  ST77XX_BLUE
#define DIGIT_ARRAY_SIZE      7
#define ESC_BAUD_RATE         256000
#define ESC_DATA_SIZE         20
#define READ_INTERVAL         0
#define ESC_TIMEOUT           10
#define DISPLAY_INTERVAL      100
#define SEALEVELPRESSURE_HPA  1013.25
#define ENABLE_BUZ            true    // enable buzzer
#define ENABLE_VIB            true    // enable vibration
#define ENABLE_VIB_LOW_BAT    true    // vibrate if armed and battery voltage sags below min volts. Gets pilot's attention.
#define MINIMUM_VOLTAGE       76.8    // 24 * 3.2V per cell
#define MAXIMUM_VOLTAGE       100.8   // 24 * 4.2V per cell
#define ALTITUDE_AGL          1       // [1] Zero altitude when armed, [0] Measure MSL altitude
#define LEFT_HAND_THROTTLE    false   // true for left handed throttle

byte escData[ESC_DATA_SIZE];
byte prevData[ESC_DATA_SIZE];
unsigned long readMillis = 0;
unsigned long transmitted = 0;
unsigned long failed = 0;
bool receiving = false;
bool armed = false;
unsigned long displayMillis = 0;
int potLvl = 0;
int throttlePWM = 0;
float throttlePercent = 0;
float prevThrotPercent = 0;
bool buttonState = 1;
bool prevButtonState = 1;
byte numberPowerCycles = 0;
float batteryPercent = 0;
float prevBatteryPercent = 0;
bool batteryFlag = true;
float armingIn = 0;
float prevArmingIn = 0;
bool throttledFlag = true;
bool throttled = false;
unsigned long throttledAtMillis = 0;
unsigned int throttleSecs = 0;
float minutes = 0;
float prevMinutes = 0;
float seconds = 0;
float prevSeconds = 0;
float hours = 0;  // logged flight hours

uint16_t _volts = 0;
uint16_t _temperatureC = 0;
int16_t _amps = 0;
uint32_t _eRPM = 0;
uint16_t _inPWM = 0;
uint16_t _outPWM = 0;

//ESC Telemetry
float volts = 0;
float prevVolts = 0;
float temperatureC = 0;
float amps = 0;
float prevAmps = 0;
float kilowatts = 0;
float prevKilowatts = 0;
float eRPM = 0;
float inPWM = 0;
float outPWM = 0;

//ALTIMETER
float ambientTempC = 0;
float prevAmbTempC = 0;
float ambientTempF = 0;
float prevAmbTempF = 0;
float pressureHpa = 0;
float altitudeM = 0;
float altitudeFt = 0;
float prevAltiFt = 0;
float altiOffsetFt = 0;
float aglFt = 0;

ResponsiveAnalogRead pot(POT_PIN, false);
Servo esc; // Creating a servo class with name of esc
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_BMP3XX bmp;
Adafruit_DRV2605 drv;
extEEPROM eep(kbits_64, 1, 32);         //device size, number of devices, page size


void setup() {
  delay(250);  // power-up safety delay
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(BTN_PIN, INPUT);
  analogReadResolution(12);     // M0 chip provides 12bit resolution
  pot.setAnalogResolution(4096);
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(0); // make sure motors off
  Serial5.begin(ESC_BAUD_RATE); 
  Serial5.setTimeout(ESC_TIMEOUT);
  buzzInit(ENABLE_BUZ);
  tftInit();
  bmpInit();
  drv.begin();
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);
  if(ENABLE_VIB) vibrateNotify();
  eepInit();
  //setFlightHours(12.6);    // uncomment to set flight log hours (0.0 to 9999.9)
  delay(1000);
}

void loop() {

  handleArming(BLACK, WHITE);
  if (armed){ 
    handleThrottle(); 
  }

  if(millis()-readMillis>=READ_INTERVAL){
    readMillis = millis();
    prepareSerialRead();
    Serial5.readBytes(escData, ESC_DATA_SIZE);
    //enforceChecksum();  //TODO when feature becomes available
    printRawSentence();
    parseData(); 
    readBMP();
  }
  
  if(armed && volts<MINIMUM_VOLTAGE){
    if(ENABLE_VIB_LOW_BAT && (millis()/1000)%2) vibrateAlert();
  }
  
  if(millis()-displayMillis>=DISPLAY_INTERVAL){
    displayMillis = millis();
    updateDisplay();
  }
}


void vibrateAlert(){
  int effect = 15; //1 through 117 (see example sketch)
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();
}


void vibrateNotify(){
  int effect = 12; //1 through 117 (see example sketch)
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();
}


void readBMP(){
  bmp.performReading();
  ambientTempC = bmp.temperature;
  ambientTempF = ambientTempC*(9/5.0)+32;
  pressureHpa = bmp.pressure;
  altitudeM = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  altitudeFt = (int)(altitudeM*3.28);
  aglFt = altitudeFt - altiOffsetFt;
}


void setAltiOffset(){
  bmp.performReading();
  ambientTempC = bmp.temperature;
  ambientTempF = ambientTempC*(9/5.0)+32;
  pressureHpa = bmp.pressure;
  altitudeM = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  altiOffsetFt = (int)(altitudeM*3.28);
}


void handleThrottle() {
  pot.update();
  potLvl = pot.getValue();
  throttlePWM = mapf(potLvl, 0, 4095, 1110, 2000); // mapping val to minimum and maximum
  throttlePercent = mapf(throttlePWM, 1112,2000, 0,100);
  if(throttlePercent<0){
    throttlePercent = 0;
  }
  esc.writeMicroseconds(throttlePWM); // using val as the signal to esc
  Serial.print(F("WRITING: "));
  Serial.println(throttlePWM);
}


void updateDisplay(){
  dispValue(volts, prevVolts, 5, 1, 84, 42, 2, BLACK, WHITE);
  tft.print("V");
  
  dispValue(amps, prevAmps, 3, 0, 108, 70, 2, BLACK, WHITE);
  tft.print("A");

  dispValue(kilowatts, prevKilowatts, 4, 1, 10, 55, 2, BLACK, WHITE);
  tft.print("kW");

  if(batteryPercent>66){
    tft.fillRect(0, 0, map(batteryPercent, 0,100, 0,108), 36, GREEN);
  }
  else if(batteryPercent>33){
    tft.fillRect(0, 0, map(batteryPercent, 0,100, 0,108), 36, YELLOW);
  }
  else{
    tft.fillRect(0, 0, map(batteryPercent, 0,100, 0,108), 36, RED);
  }
  if(volts < MINIMUM_VOLTAGE){
    if(batteryFlag){
      batteryFlag = false;
      tft.fillRect(0, 0, 108, 36, WHITE);
    }
    tft.setCursor(0,3);
    tft.setTextSize(2);
    tft.setTextColor(RED);
    tft.println(" BATTERY");
    tft.print("  DEAD!");
  }
  else{
    batteryFlag = true;
    tft.fillRect(map(batteryPercent, 0,100, 0,108), 0, map(batteryPercent, 0,100, 108,0), 36, WHITE);
  }
  dispValue(batteryPercent, prevBatteryPercent, 3, 0, 108, 10, 2, BLACK, WHITE);
  tft.print("%");

  tft.fillRect(0, 36, 160, 1, BLACK);
  tft.fillRect(108, 0, 1, 36, BLACK);
  tft.fillRect(0, 92, 160, 1, BLACK);
  
  if(ALTITUDE_AGL) dispValue(aglFt, prevAltiFt, 5, 0, 70, 102, 2, BLACK, WHITE);
  else dispValue(altitudeFt, prevAltiFt, 5, 0, 70, 102, 2, BLACK, WHITE);
  tft.print("ft");
  
  //dispValue(ambientTempF, prevAmbTempF, 3, 0, 10, 100, 2, BLACK, WHITE);
  //tft.print("F");

  handleFlightTime();
  displayTime(throttleSecs, 8, 102);
  
  //dispPowerCycles(104,100,2);
}


void handleFlightTime(){
  if(!armed){
    throttledFlag = true;
    throttled = false;
  }
  if(armed){
    if(throttlePercent>50 && throttledFlag){
      throttledAtMillis = millis();
      throttledFlag = false;
      throttled = true;
    }
    if(throttled){
      throttleSecs = (millis()-throttledAtMillis) / 1000.0;
    }
    else{
      throttleSecs = 0;
    }
  }
}


void displayTime(int val, int x, int y) {
  // displays number of minutes and seconds (since armed and throttled)
  tft.setCursor(x,y);
  tft.setTextSize(2);
  tft.setTextColor(BLACK);
  minutes = val / 60;
  seconds = numberOfSeconds(val);
  if (minutes < 10) {
    tft.setCursor(x,y);
    tft.print("0");
  }
  dispValue(minutes, prevMinutes, 2, 0, x, y, 2, BLACK, WHITE);
  tft.setCursor(x+24,y);
  tft.print(":");
  tft.setCursor(x+36,y);
  if (seconds < 10) {
    tft.print("0");
  }
  dispValue(seconds, prevSeconds, 2, 0, x+36, y, 2, BLACK, WHITE);
}


void dispPowerCycles(int x, int y, int textSize){
  int maxDigits = 3;
  int width = 6*textSize;
  int height = 8*textSize;
  tft.fillRect(x, y, width*maxDigits, height, ST77XX_BLACK);
  tft.setCursor(x,y);
  tft.setTextSize(textSize);
  tft.print(numberPowerCycles);
}


//**************************************************************************************//
//  Helper function to print values without flashing numbers due to slow screen refresh.
//  This function only re-draws the digit that needs to be updated.
//    BUG:  If textColor is not constant across multiple uses of this function,
//          weird things happen.
//**************************************************************************************//
void dispValue(float &value, float &prevVal, int maxDigits, int precision, int x, int y, int textSize, int textColor, int background){
  int numDigits = 0;
  char prevDigit[DIGIT_ARRAY_SIZE] = {};
  char digit[DIGIT_ARRAY_SIZE] = {};
  String prevValTxt = String(prevVal, precision);
  String valTxt = String(value, precision);
  prevValTxt.toCharArray(prevDigit, maxDigits+1);
  valTxt.toCharArray(digit, maxDigits+1);

  // COUNT THE NUMBER OF DIGITS THAT NEED TO BE PRINTED:
  for(int i=0; i<maxDigits; i++){
    if(digit[i]){
      numDigits++;
    }
  }
  
  //tft.fillScreen(ST77XX_WHITE);  // before
  
  tft.setTextSize(textSize);
  tft.setCursor(x,y);
  
  // PRINT LEADING SPACES TO RIGHT-ALIGN:
  for(int i=0; i<(maxDigits-numDigits); i++){
    tft.setTextColor(background);
    tft.print(char(218));
    tft.setTextColor(textColor);
  }
  
  // ERASE ONLY THE NESSESARY DIGITS:
  for(int i=0; i<numDigits; i++){
    if(digit[i]!=prevDigit[i]){
      tft.setTextColor(background);
      tft.print(char(218));
      tft.setTextColor(textColor);
    }
    else{
      tft.print(digit[i]);
    }
  }
  
  // BACK TO THE BEGINNING:
  tft.setCursor(x,y);

  // ADVANCE THE CURSOR TO THE PROPER LOCATION:
  for(int i=0; i<(maxDigits-numDigits); i++){
    tft.setTextColor(background);
    tft.print(char(218));
    tft.setTextColor(textColor);
  }

  // PRINT THE DIGITS THAT NEED UPDATING
  for(int i=0; i<numDigits; i++){
    tft.print(digit[i]);
  }

  prevVal = value;
}


void handleArming(int textColor, int background){
  if(digitalRead(BTN_PIN)==LOW){
    bool armReady = false;
    tft.fillScreen(background);
    tft.setTextColor(textColor);
    tft.setCursor(4,10);
    if(!armed){tft.print(F(" ARMING IN: "));}
    else{tft.print(F("DISARMING IN: "));}
    tft.setCursor(10,105);
    tft.print(F("Log: "));
    if(hours>=1000) tft.print(hours,0);
    else tft.print(hours,1);
    tft.print(F("hr"));
    unsigned long prePressMillis = millis();
    while(digitalRead(BTN_PIN)==LOW && !armReady){
      delay(100);
      armingIn = 3-((millis()-prePressMillis)/1000);
      //dispArmingIn(armingIn, 0, 20, 4);
      dispValue(armingIn, prevArmingIn, 2, 0, 20, 40, 7, BLUE, WHITE);
      if(armingIn<1){
        armReady = true;
      }
      if(armReady || digitalRead(BTN_PIN)==HIGH){
        tft.fillScreen(background);
        //dispAltiPortion(20,70,2);
      }
    }
    //tft.fillScreen(ST77XX_BLACK);
    unsigned long postPressMillis = millis();
    if(postPressMillis-prePressMillis>3000){
      if(!armed){
        armSystem();
      }
      else if(armed){
        disarmSystem();
      }
    }
  }
}


void dispArmingIn(int _armingIn, int x, int y, int textSize){
  int maxDigits = 1;
  int width = 6*textSize;
  int height = 8*textSize;
  tft.fillRect(x, y, width*maxDigits, height, ST77XX_RED);
  tft.setCursor(x,y);
  tft.setTextSize(textSize);
  tft.print(_armingIn);
}


void armSystem(){
  Serial.println(F("Sending Arm Signal"));
  esc.writeMicroseconds(1000); // initialize the signal to 1000
  setAltiOffset();
  armed = true;
  digitalWrite(LED_BUILTIN, HIGH);
  if(ENABLE_VIB) vibrateNotify();
  delay(500);
  if(ENABLE_BUZ){
    tone(BUZ_PIN, 500, 250);
    delay(250);
    tone(BUZ_PIN, 700, 250);
  }
}


void disarmSystem() {
  esc.writeMicroseconds(0);
  recordFlightHours();
  
  armed = false;
  Serial.println(F("disarmed"));
  digitalWrite(LED_BUILTIN, LOW);
  if(ENABLE_VIB) vibrateNotify();
  delay(500);
  if(ENABLE_BUZ){
    tone(BUZ_PIN, 700, 250);
    delay(250);
    tone(BUZ_PIN, 500, 250);
  }
  delay(1000); // dont allow immediate rearming
}


void eepInit(){
    eep.begin();
  for(int i=0; i<6; i++){
    //eep.write(i,0);         //uncomment to reset log to zero
  }
  //  eep.write(0,5);           // This is to set flight log minutes
  //  eep.write(1,9);           // Example here: 599,940 minutes
  //  eep.write(2,9);
  //  eep.write(3,9);
  //  eep.write(4,4);
  //  eep.write(5,0);
  int LogMin[6];
  for(int i=0; i<6; i++){
    LogMin[i] = eep.read(i);
  }
  float logMinutes = float(LogMin[0]*100000 + LogMin[1]*10000 + LogMin[2]*1000 
                      + LogMin[3]*100 + LogMin[4]*10 + LogMin[5]);
  hours = logMinutes / 60.0;
}


void recordFlightHours(){
  int newLogMinutes = hours*60.0 + throttleSecs/60.0; // 599940
  Serial.print(F("newLogMinutes: "));
  Serial.println(newLogMinutes);
  int LogMin[6];
  LogMin[0] = newLogMinutes / 100000;                                                                                       // 599940 / 100000 = 5
  LogMin[1] = (newLogMinutes - LogMin[0]*100000) / 10000;                                                                   // (599940 - 5*100000) / 10000 = 9
  LogMin[2] = (newLogMinutes - LogMin[0]*100000 - LogMin[1]*10000) / 1000;                                                  // (599940 - 5*100000 - 9*10000) / 1000 = 9
  LogMin[3] = (newLogMinutes - LogMin[0]*100000 - LogMin[1]*10000 - LogMin[2]*1000) / 100;                                  // (599940 - 5*100000 - 9*10000 - 9*1000) / 100 = 9
  LogMin[4] = (newLogMinutes - LogMin[0]*100000 - LogMin[1]*10000 - LogMin[2]*1000 - LogMin[3]*100) / 10;                   // (599940 - 5*100000 - 9*10000 - 9*1000 - 9*100) / 10 = 4
  LogMin[5] = (newLogMinutes - LogMin[0]*100000 - LogMin[1]*10000 - LogMin[2]*1000 - LogMin[3]*100 - LogMin[4]*10);         // (599940 - 5*100000 - 9*10000 - 9*1000 - 9*100 - 4*10) = 0
  
  for(int i=0; i<6; i++){
    eep.write(i, LogMin[i]+0);
    Serial.print(F("LogMin["));
    Serial.print(i);
    Serial.print(F("]: "));
    Serial.println(LogMin[i]+0);
  }
  for(int i=0; i<6; i++){
    LogMin[i] = eep.read(i);
  }
  float logMinutes = float(((LogMin[0])*100000) + ((LogMin[1])*10000) 
  + ((LogMin[2])*1000) + ((LogMin[3])*100) + ((LogMin[4])*10) + (LogMin[5]));
  hours = logMinutes / 60.0;
}


void setFlightHours(float hr){
  int newLogMinutes = hr*60.0; // 599940
  Serial.print(F("newLogMinutes: "));
  Serial.println(newLogMinutes);
  int LogMin[6];
  LogMin[0] = newLogMinutes / 100000;                                                                                       // 599940 / 100000 = 5
  LogMin[1] = (newLogMinutes - LogMin[0]*100000) / 10000;                                                                   // (599940 - 5*100000) / 10000 = 9
  LogMin[2] = (newLogMinutes - LogMin[0]*100000 - LogMin[1]*10000) / 1000;                                                  // (599940 - 5*100000 - 9*10000) / 1000 = 9
  LogMin[3] = (newLogMinutes - LogMin[0]*100000 - LogMin[1]*10000 - LogMin[2]*1000) / 100;                                  // (599940 - 5*100000 - 9*10000 - 9*1000) / 100 = 9
  LogMin[4] = (newLogMinutes - LogMin[0]*100000 - LogMin[1]*10000 - LogMin[2]*1000 - LogMin[3]*100) / 10;                   // (599940 - 5*100000 - 9*10000 - 9*1000 - 9*100) / 10 = 4
  LogMin[5] = (newLogMinutes - LogMin[0]*100000 - LogMin[1]*10000 - LogMin[2]*1000 - LogMin[3]*100 - LogMin[4]*10);         // (599940 - 5*100000 - 9*10000 - 9*1000 - 9*100 - 4*10) = 0
  
  for(int i=0; i<6; i++){
    eep.write(i, LogMin[i]+0);
    Serial.print(F("LogMin["));
    Serial.print(i);
    Serial.print(F("]: "));
    Serial.println(LogMin[i]+0);
  }
  for(int i=0; i<6; i++){
    LogMin[i] = eep.read(i);
  }
  float logMinutes = float(((LogMin[0])*100000) + ((LogMin[1])*10000) 
  + ((LogMin[2])*1000) + ((LogMin[3])*100) + ((LogMin[4])*10) + (LogMin[5]));
  hours = logMinutes / 60.0;
}


void bmpInit(){
  bmp.begin();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
}


void buzzInit(bool enableBuz){
  pinMode(BUZ_PIN, OUTPUT);
  if(enableBuz){
    tone(BUZ_PIN, 500);
    delay(200);
    tone(BUZ_PIN, 700);
    delay(200);
    tone(BUZ_PIN, 900);
    delay(200);
    noTone(BUZ_PIN);
  }
}


void tftInit(){
  tft.initR(INITR_BLACKTAB);          // Init ST7735S chip, black tab
  tft.fillScreen(WHITE);
  tft.setTextColor(BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(1);
  tft.setTextWrap(true);
  int rotation = 1;
  if(LEFT_HAND_THROTTLE) rotation = 3;
  tft.setRotation(rotation); // 1=right hand, 3=left hand
  pinMode(TFT_LITE, OUTPUT);
  digitalWrite(TFT_LITE, HIGH);  // Backlight on
}


void prepareSerialRead(){  
  while(Serial5.available()>0){
    byte t = Serial5.read();
  }
}


void enforceChecksum(){
  //Check checksum, revert to previous data if bad:
  word checksum = word(escData[19], escData[18]);
  int sum = 0;
  for(int i=0; i<ESC_DATA_SIZE-2; i++){
    sum += escData[i];
  }
  Serial.print(F("     SUM: "));
  Serial.println(sum);
  Serial.print(F("CHECKSUM: "));
  Serial.println(checksum);
  if(sum != checksum){
    Serial.println(F("_________________________________________________CHECKSUM FAILED!"));
    failed++;
    if(failed>=1000) {  // keep track of how reliable the transmission is
      transmitted = 1;
      failed = 0;
    }
    for(int i=0; i<ESC_DATA_SIZE; i++){  // revert to previous data
      escData[i] = prevData[i];
    }
  }
  for(int i=0; i<ESC_DATA_SIZE; i++){
    prevData[i] = escData[i];
  }
}


void printRawSentence(){
  Serial.print(F("DATA: "));
  for(int i=0; i<ESC_DATA_SIZE; i++){
    Serial.print(escData[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}


void parseData(){
  // LSB First
  
  _volts = word(escData[1], escData[0]);
  //_volts = ((unsigned int)escData[1] << 8) + escData[0];
  volts = _volts/100.0;
  //reading 23.00 = 22.7 actual
  //reading 16.00 = 15.17 actual
  Serial.print(F("Volts: "));
  Serial.println(volts);

  batteryPercent = mapf(volts, MINIMUM_VOLTAGE, MAXIMUM_VOLTAGE, 0.0, 100.0);
  if(batteryPercent < 0){ 
    batteryPercent = 0;
  }
  if(batteryPercent >100){
    batteryPercent = 100;
  }
  
  _temperatureC = word(escData[3], escData[2]);
  temperatureC = _temperatureC/100.0;
  //reading 17.4C = 63.32F in 84F ambient?
  Serial.print(F("TemperatureC: "));
  Serial.println(temperatureC);

  _amps = word(escData[5], escData[4]);
  amps = _amps/10.0;
  Serial.print(F("Amps: "));
  Serial.println(amps);

  kilowatts = amps*volts/1000.0;

  // 7 and 6 are reserved bytes
  
  _eRPM = escData[11];     // 0 
  _eRPM << 8;
  _eRPM += escData[10];    // 0
  _eRPM << 8;
  _eRPM += escData[9];     // 30
  _eRPM << 8;
  _eRPM += escData[8];     // b4
  eRPM = _eRPM/6.0/2.0;
  Serial.print(F("eRPM: "));
  Serial.println(eRPM);
  
  _inPWM = word(escData[13], escData[12]);
  inPWM = _inPWM/100.0;
  Serial.print(F("inPWM: "));
  Serial.println(inPWM);

  _outPWM = word(escData[15], escData[14]);
  outPWM = _outPWM/100.0;
  Serial.print(F("outPWM: "));
  Serial.println(outPWM);
  
  // 17 and 16 are reserved bytes
  // 19 and 18 is checksum
  word checksum = word(escData[19], escData[18]);
  Serial.print(F("CHECKSUM: "));
  Serial.print(escData[19]);
  Serial.print(F(" + "));
  Serial.print(escData[18]);
  Serial.print(F(" = "));
  Serial.println(checksum);

  Serial.print(F("hours: "));
  Serial.println(hours);
  Serial.print(F("throttleSecs: "));
  Serial.println(throttleSecs);
  
}


double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
