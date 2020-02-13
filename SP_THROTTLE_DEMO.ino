#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
//#include <Adafruit_ZeroDMA.h>
#include <Servo.h> // to control ESCs
#include <Adafruit_BMP3XX.h>
#include <Adafruit_DRV2605.h>
#include <extEEPROM.h>

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
#define DIGIT_ARRAY_SIZE      7
#define ESC_BAUD_RATE         256000
#define ESC_DATA_SIZE         20
#define READ_INTERVAL         0
#define ESC_TIMEOUT           10
#define DISPLAY_INTERVAL      100
#define SEALEVELPRESSURE_HPA  1013.25
#define ENABLE_BUZ            false

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
  vibrateNotify();
  eep.begin();
  //eep.write(0, 0);  //uncomment to reset numberPowerCycles
  numberPowerCycles = eep.read(0);
  eep.write(0, ++numberPowerCycles);
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
    //enforceChecksum();  //TODO
    printRawSentence();
    parseData(); 
    readBMP();
  }
  
  if(volts<43){
    //vibrateAlert();
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
  dispValue(volts, prevVolts, 4, 1, 96, 42, 2, BLACK, WHITE);
  tft.print("V");
  
  dispValue(amps, prevAmps, 3, 0, 108, 70, 2, BLACK, WHITE);
  tft.print("A");

  dispValue(kilowatts, prevKilowatts, 4, 1, 10, 55, 2, BLACK, WHITE);
  tft.print("kW");

  if(throttlePercent>66){
    tft.fillRect(0, 0, map(throttlePercent, 0,100, 0,108), 36, GREEN);
  }
  else if(throttlePercent>33){
    tft.fillRect(0, 0, map(throttlePercent, 0,100, 0,108), 36, YELLOW);
  }
  else{
    tft.fillRect(0, 0, map(throttlePercent, 0,100, 0,108), 36, RED);
  }
  tft.fillRect(map(throttlePercent, 0,100, 0,108), 0, map(throttlePercent, 0,100, 108,0), 36, WHITE);
  dispValue(throttlePercent, prevThrotPercent, 3, 0, 108/*55*/, 10, 2, BLACK, WHITE);
  tft.print("%");

  tft.fillRect(0, 36, 160, 1, BLACK);
  tft.fillRect(108, 0, 1, 36, BLACK);
  tft.fillRect(0, 92, 160, 1, BLACK);
  
  dispValue(altitudeFt, prevAltiFt, 5, 0, 70, 100, 2, BLACK, WHITE);
  tft.print("ft");
  
  dispValue(ambientTempF, prevAmbTempF, 3, 0, 10, 100, 2, BLACK, WHITE);
  tft.print("F");
  
  //dispPowerCycles(104,100,2);
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
    tft.setCursor(0,0);
    if(!armed){tft.print(F("ARMING IN: "));}
    else{tft.print(F("DISARMING IN: "));}
    unsigned long prePressMillis = millis();
    while(digitalRead(BTN_PIN)==LOW && !armReady){
      delay(100);
      int armingIn = 4-((millis()-prePressMillis)/1000.0);
      dispArmingIn(armingIn, 0, 20, 4);
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
  armed = true;
  digitalWrite(LED_BUILTIN, HIGH);
  vibrateNotify();
  delay(500);
  if(ENABLE_BUZ){
    tone(BUZ_PIN, 500, 250);
    delay(250);
    tone(BUZ_PIN, 700, 250);
  }
}


void disarmSystem() {
  esc.writeMicroseconds(0);
  armed = false;
  Serial.println(F("disarmed"));
  digitalWrite(LED_BUILTIN, LOW);
  vibrateNotify();
  delay(500);
  if(ENABLE_BUZ){
    tone(BUZ_PIN, 700, 250);
    delay(250);
    tone(BUZ_PIN, 500, 250);
  }
  delay(1000); // dont allow immediate rearming
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
  tft.setRotation(1);
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
  Serial.print("     SUM: ");
  Serial.println(sum);
  Serial.print("CHECKSUM: ");
  Serial.println(checksum);
  if(sum != checksum){
    Serial.println("_________________________________________________CHECKSUM FAILED!");
    failed++;
    if(failed>=1000) {
    transmitted = 1;
    failed = 0;
  }
    for(int i=0; i<ESC_DATA_SIZE; i++){
      escData[i] = prevData[i];
    }
  }
  for(int i=0; i<ESC_DATA_SIZE; i++){
    prevData[i] = escData[i];
  }
}


void printRawSentence(){
  Serial.print("DATA: ");
  for(int i=0; i<ESC_DATA_SIZE; i++){
    Serial.print(escData[i], HEX);
    Serial.print(" ");
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
  Serial.print("Volts: ");
  Serial.println(volts);
  
  _temperatureC = word(escData[3], escData[2]);
  temperatureC = _temperatureC/100.0;
  //reading 17.4C = 63.32F in 84F ambient?
  Serial.print("TemperatureC: ");
  Serial.println(temperatureC);

  _amps = word(escData[5], escData[4]);
  amps = _amps/10.0;
  Serial.print("Amps: ");
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
  Serial.print("eRPM: ");
  Serial.println(eRPM);
  
  _inPWM = word(escData[13], escData[12]);
  inPWM = _inPWM/100.0;
  Serial.print("inPWM: ");
  Serial.println(inPWM);

  _outPWM = word(escData[15], escData[14]);
  outPWM = _outPWM/100.0;
  Serial.print("outPWM: ");
  Serial.println(outPWM);
  
  // 17 and 16 are reserved bytes
  // 19 and 18 is checksum
  
}


double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
