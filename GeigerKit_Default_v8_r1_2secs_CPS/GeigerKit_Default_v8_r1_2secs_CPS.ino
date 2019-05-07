/* GeigerKit_Default sketch (v.8.1)                                           bHogan 6/22/12
 * This sketch was written for the DIYGeigerCounter Kit.
 * DIY Geiger invests a lot time and resources in providing this open source code, 
 * please support it by not purchasing knock-off versions of the hardware.
 * It requires the Arduino IDE rel. 1.0.0 or above to compile.
 *
 * FEATURES:
 * CPM & uSv/hr output to LCD display and serial port. Bar graph on LCD. Low voltage warning.
 * Counts are displayed every 5 sec using a moving average which is reset every 30 sec. 
 * An alternate display shows a running average for the previous 1 and 10 minute periods.
 * One of 2 CPM to uSv/h conversion ratios can be selected via jumper. ACCURACY NOT VERIFIED
 * This version adds a configurable alarm, new alt screen, and a 'realtime' bargraph.
 * It also supports the SensorGraph app on the Android.
 * More info is available at: http://sites.google.com/site/diygeigercounter/home
 * v.8.1 display fixes - had to reduce bargraph from 7 to 6 char for display sanity
 * SETUP: (Any 2x16 Hitachi HD44780 compatible LCD)
 *     +5V LCD Vdd pin 2             >>>     Gnd LCD Vss pin 1, and R/W pin 5
 *     LCD RS pin 4 to D3            >>>     LCD Enable pin 6 to D4
 *     LCD D4 pin 11 to D5           >>>     LCD D5 pin 12 to D6
 *     LCD D6 pin 13 to D7           >>>     LCD D7 pin 14 to D8
 *     LCD LEDA pin 15 to ~1K to +5  >>>     LCD LEDK pin 16 to GND
 *     10K pot: - ends to +5V and Gnd, wiper to LCD VO pin (pin 3)
 * *INT from Geiger circuit is connected to PIN 2 and triggered as FALLING.
 * PIN 9 - jumper to GND if secondary conversion ratio is desired
 * PIN 10 - use button for alt display and to set alarm. (see site)
 *
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2.1 of the License, or any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 * Do not remove information from this header.
 * 
 * THIS PROGRAM AND IT'S MEASUREMENTS IS NOT INTENDED TO GUIDE ACTIONS TO TAKE, OR NOT
 * TO TAKE, REGARDING EXPOSURE TO RADIATION. THE GEIGER KIT AND IT'S SOFTWARE ARE FOR
 * EDUCATIONAL PURPOSES ONLY. DO NOT RELY ON THEM IN HAZARDOUS SITUATIONS!
 */

#include <MeetAndroid.h>                // connects Geiger to SensorGraph on Android
#include <LiquidCrystal.h>              // HD44780 compatible LCDs work with this lib
#include <EEPROM.h>                     // alarm setting is stored in EEPROM
#define DEBUG         false             // if true, shows available memory
#define TUBE_SEL          9             // jumper to select alt conversion to uSv
#define BUTTON_PIN       10             // button to toggle alternate display and set alarm
#define LED_PIN          13             // for debug only - flashes 5X at startup
#define ALARM_PIN        15             // Outputs HIGH when Alarm triggered
#define ALARM_SET_ADDR    8             // address of alarm setting in EEPROM
#define MAX_ALARM        600            // max the alarm can be set for

// HOW MANY CPM =1 uSV? The commonly used ratios for the SBM-20 & LND712 are defined:
#define PRI_RATIO     175.43            // no TUBE_SEL jumper - SET FOR SBM-20
#define SEC_RATIO     100.00            // TUBE_SEL jumper to GND - SET FOR LND712

// mS between writes to serial - counts also accumulate seperately for this period
#define LOGGING_PEROID   1000.0         // 1 sec (1000 mS) best
// mS between writes to display - counts/period are averaged to CPM
#define DISP_PERIOD      1000.0         // (1 sec) mS sample & display
#define FULL_SCALE           35         // max CPS for all 6 bars
#define LOW_VCC            4200 //mV    // if Vcc < LOW_VCC give low voltage warning
#define ONE_MIN_MAX          12         // elements in the oneMinute accumulater array
#define TEN_MIN_MAX         120         // elements in the tenMinute accumulater array
#define DEBOUNCE_MS          50         // buttom debounce period in mS

boolean lowVcc = false;                 // true when Vcc < LOW_VCC
boolean altDispUsed = false;            // used to start counts from zero
boolean altDispOn = false;              // true when SW_1 on, and in continous count mode
boolean dispOneMin = false;             // true if enough time passed to begin displaying counts
boolean dispTenMin = false;
boolean AlarmOn = false;                // CPM > set alarm
byte sampleCnt;                         // the number of samples making up the average
byte altDispCnt = 0;                    // controls when alt display is on
int oneMinute[ONE_MIN_MAX];             // used to accumulate 1 & 10 minute counts
int oneMinuteIndex = 0;
int tenMinute[TEN_MIN_MAX];
int tenMinuteIndex = 0;
int AlarmPoint;                         // CPM alarm is set for
int Vcc_mV;                             // mV of Vcc from last check 
int androidReturn = 0;                  // reads the slider on Android SensorGraph.

volatile unsigned long dispCnt, logCnt; // inc by ISR
volatile unsigned long runCnt, barCnt;
long runCountStart;
unsigned long tempSum;
unsigned long dispPeriodStart, dispCPM; // counters for the display period . . .
unsigned long logPeriodStart, logCPM;   
unsigned long checkVccTime;             // counter for how often to check voltage
unsigned long altDispStart;             // counter for alt display refresh period
unsigned long lastButtonTime;           // counter for pressing the button to quickly
unsigned long barGraphStart;            // counter for bargraph refresh period
float uSv = 0.0;                        // display CPM converted to "unofficial" uSv
float uSvLogged = 0.0;                  // logging CPM converted to "unofficial" uSv
float uSvRate;                          // holds the rate selected by jumper
float avgCnt = 0.0;                     // holds the previous average count
float temp_uSv = 0.0;

//Custom characters used for bar graph
byte bar_0[8] = {
  0x00, 0x00, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00}; //blank
byte bar_1[8] = {
  0x10, 0x10, 0x18, 0x18, 0x18, 0x10, 0x10, 0x00}; //1 bar
byte bar_2[8] = {
  0x18, 0x18, 0x1c, 0x1c, 0x1c, 0x18, 0x18, 0x00}; //2 bars
byte bar_3[8] = {
  0x1C, 0x1C, 0x1e, 0x1e, 0x1e, 0x1C, 0x1C, 0x00}; //3 bars
byte bar_4[8] = {
  0x1E, 0x1E, 0x1f, 0x1f, 0x1f, 0x1E, 0x1E, 0x00}; //4 bars
byte bar_5[8] = {
  0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00}; //5 bars
byte bar_6[8] = {
  0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00}; //6 bars (same)

// instantiate the library and pass pins for (RS, Enable, D4, D5, D6, D7)
LiquidCrystal lcd(3, 4, 5, 6, 7, 8);    // default layout for the Geiger board 
MeetAndroid Android;

void setup(){
  Serial.begin(9600);                   // comspec 96,N,8,1
  attachInterrupt(0,GetEvent,FALLING);  // Geiger event on pin 2 triggers interrupt
  pinMode(LED_PIN,OUTPUT);              // setup LED pin
  pinMode(TUBE_SEL,INPUT);              // setup tube select jumper pin
  pinMode(BUTTON_PIN,INPUT);            // setup menu button
  pinMode(ALARM_PIN, OUTPUT);           // setup Alarm pin
  digitalWrite(TUBE_SEL, HIGH);         // set 20K pullup on jumper pins(low active)
  digitalWrite(BUTTON_PIN, HIGH);
  Blink(LED_PIN,4);                     // show it's alive
  lcd.begin(16,2);                      // cols, rows of display (8x2, 16x2, etc.)
  lcd.createChar(0, bar_0);             // load 7 custom characters in the LCD
  lcd.createChar(1, bar_1);
  lcd.createChar(2, bar_2);
  lcd.createChar(3, bar_3);
  lcd.createChar(4, bar_4);
  lcd.createChar(5, bar_5);
  lcd.createChar(6, bar_6);
  lcd.setCursor(0,0);

  Android.registerFunction(androidInput,'o'); // calls androidInput() when new input frpm droid
  clearDisp();                          // clear the screen
  lcd.print("   GEIGER KIT");           // display a simple banner
  lcd.setCursor(0,1);                   // set cursor on line 2
  lcd.print("    Ver. 8.0");            // display the version
  delay (1500);                         // leave the banner up for a bit
  clearDisp();                          // clear the screen
  if(digitalRead(TUBE_SEL)){            // read jumper to select conversion ratio
    uSvRate = PRI_RATIO;                // use the primary ratio defined
  }
  else{                                 // jumper is set to GND . . .
    uSvRate = SEC_RATIO;                // use the secondary ratio defined
  }
  lcd.print(uSvRate,0);                 // display conversion ratio in use 
  lcd.print(" CPM to \xe4Sv");          // \xe4 = Mu char
  lcd.setCursor(0,1);                   // set cursor on line 2
  Vcc_mV = readVcc();                   // read Vcc voltage
  lcd.print("Running at ");             // display it
  lcd.print(Vcc_mV/1000. ,2);           // display as volts with 2 dec. places
  lcd.print("V");
  delay (2000);                         // leave info up for a bit
#if (DEBUG)                             // show available SRAM if DEBUG true
  clearDisp();
  lcd.print("RAM Avail: ");
  lcd.print(AvailRam());
  delay (2000);
#endif
  // this section tests if button pressed to set alarm threshold and calls a function to change it
  AlarmPoint = EEPROM.read(ALARM_SET_ADDR); // get last alarm value from EEPROM
  if (AlarmPoint == 255) AlarmPoint = 0;    // deal with virgin EEPROM
  AlarmPoint = AlarmPoint * 10;             // values stored are 1/10th
  clearDisp();                              // put up new alarm set screen
  lcd.print("Set Alarm?");
  lcd.setCursor(0, 1);
  if (AlarmPoint >0){
    lcd.print("Now "); 
    lcd.print(AlarmPoint); 
    lcd.print(" CPM"); 
  }
  else lcd.print("Now Off"); 

  long timeIn = millis();               // you have 2 sec to push button or move on
  while (millis() < timeIn + 2000) { 
    if (readButton(BUTTON_PIN)== LOW) setAlarm(); // alarm is to be set
  }                                     // if no button press continue
  clearDisp();                          // clear the screen
  lcd.print("CPS? ");                   // display beginning "CPM"
//  Serial.println("CPM \t uSv \t Vcc");  // print header for log - uSv removed. tabs removed. DC
  Serial.println("CPS Vcc");  // print header for log
  dispPeriodStart = millis();           // start timing display CPM
  logPeriodStart = dispPeriodStart;     // start logging timer
  checkVccTime = dispPeriodStart;       // start Vcc timer
  runCountStart = dispPeriodStart;      // start alternate timer
  barGraphStart = dispPeriodStart;      // start bargraph timer
  dispCnt = 0;                          // start with fresh totals
  logCnt= 0;
  runCnt = 0;
  barCnt = 0;
}


void loop(){
  //uncomment 5 lines below for self check - you should see very close to 360 CPM
  //dispCnt++;
  //logCnt++;
  //runCnt++;
  //barCnt++;
  //delay(167);                           // 167 mS = 6 Hz `= X 60 = 360 CPM  

  if (millis() >= lastButtonTime + 500){ // wait a bit between button pushes
    lastButtonTime = millis();          // reset the period time
    if (readButton(BUTTON_PIN)== LOW){  // start alt display mode if button pin is low
      clearDisp();                      // clear the screen when switching displays
      altDispOn = !altDispOn;           // toggle altDispOn state
      if (altDispUsed == false){
        altDispUsed = true;             // false until first used
        runCnt = 0;                     // clear counts until used
        runCountStart = millis();
      }
      if (altDispOn) DispRunCounts();   // start alt display immediately - or
      else DispCounts(dispCnt);         // start main display immediately
    }
  }

  if (millis() >= barGraphStart + 1000){ // refresh bargraph, alarm and Vcc if in main display
    if (!altDispOn)fastDisplay();       // display quick response data       
    barCnt = 0;                         // reset counts
    barGraphStart = millis();           // reset the period time
  }

  if (millis() >= altDispStart + 1000){ // refresh alt display if it's on
    if (altDispOn) DispRunCounts();     // display running counts
    altDispStart = millis();            // reset the period time
  }

  if (millis() >= runCountStart + 1000){ // Collect running counts every 1 sec.
    RunningCounts(runCnt);              // add counts
    runCnt = 0;                         // reset counts
    runCountStart = millis();           // reset the period time
  }

  if (millis() >= dispPeriodStart + DISP_PERIOD){ // DISPLAY PERIOD
    if (!altDispOn) DispCounts(dispCnt);// period is over - display counts
    dispCnt = 0;                        // reset counter
    dispPeriodStart = millis();         // reset the period time
  }

  if (millis() >= logPeriodStart + LOGGING_PEROID){ // LOGGING PERIOD
    logCount(logCnt);                   // pass in the counts to be logged
    logCnt = 0;                         // reset log event counter
    dispCnt = 0;                        // reset display event counter too
    dispPeriodStart = millis();         // reset display time too
    logPeriodStart = millis();          // reset log time
  }

  if (millis() >= checkVccTime + 1000){ // timer for check battery (every 6 sec.)
    checkVccTime = millis();            // reset timer
    Vcc_mV = readVcc();
    if (Vcc_mV <= LOW_VCC) lowVcc = true; // check if Vcc is low 
    else lowVcc = false;
  }
}


void DispCounts(long dcnt){  // calc and display predicted CPM & uSv/h
  byte maxSamples = (60000 / DISP_PERIOD) / 2;   // number of sample periods in 30 seconds                     
  sampleCnt++;                                   // inc sample count - must be at least 1
  avgCnt += (dcnt - avgCnt) / sampleCnt;         // CALCULATE AVERAGE COUNT - moving average
//  dispCPM = avgCnt;    // convert to CPM - removed for next line - DC
  dispCPM = dcnt; // pass count in each second directly to LCD display
  //handle reset of sample count - sample is for 1 msec and reset. Options for reset value are:
  // "0" - throw away last average, "1" - keeps last average, "maxSamples -1" - keeps running avg.
  if (sampleCnt >= maxSamples) sampleCnt = 0;    // start a fresh average every 30 sec.
  // the following line gives a faster response when counts increase or decrease rapidly 
  if ((dcnt - avgCnt) > 9 || (avgCnt - dcnt) > 9) sampleCnt = 0;
  uSv = dispCPM / uSvRate;                       // make uSV conversion
  //Blink(LED_PIN,1);                              // uncomment to blink each didplay
  
  Android.receive();                    // looks for new input from Android
  // don't send data via BT if not using app (no input) else it screws up serial output
  if (androidReturn >0)Android.send((dispCPM * androidReturn) / 100);

  clearArea (0,0,10);                   // clear count area
  lcd.print("CPS ");                    // display static "CPM"
  clearArea (0,1,16);                   // clear line 2
  lcd.print("\xe4Sv/hr ");              // display static "uSv/hr"
  lcd.print("NIL");                     // dose rate not applicable - DC
//  lcd.print(uSv,2);                     // display uSv/hr on line 1
  lcd.setCursor(4,0);                   // CPM LAST TO DISPLAY - NEVER PARTIALLY OVERWRITTEN
  lcd.print(dispCPM);                   // display CPM on line 1
  lcd.print(" ");
  if (dispCPM > AlarmPoint && AlarmPoint > 0) {  // Alarm takes priority over low Vcc
    AlarmOn = true;
    digitalWrite(ALARM_PIN, HIGH);      // turn on alarm (set alarm pin to Vcc) 
  }
  else {
    digitalWrite(ALARM_PIN, LOW);       // turn off alarm (set alarm pin to Gnd)
    AlarmOn = false;  
  }
}


void fastDisplay(){ // quick response display on 2nd half of line 1
  barCnt = barCnt;                 // scale CPS to CPM Scale removed for 1 sec refresh

  if (barCnt <= FULL_SCALE && !AlarmOn){
    clearArea (10,0,6);                 // move cursor to 9th col, 1st line for lcd bar
    lcdBar(barCnt);                     // display bargraph on line 1
  }
  if (lowVcc) {                         // overwrite display with battery voltage if low
    clearArea (11,0,5);
    lcd.print(Vcc_mV/1000.,2);          // display as volts with 2 dec. place
    lcd.print("V");
  }
  if (AlarmOn) {                        // overwrite display with alarm if on
    clearArea (10,0,6);
    lcd.setCursor(11,0);
    lcd.print("ALARM");
  } 
}


void DispRunCounts(){ // create the screen that shows the running counts
  clearDisp();
  lcd.print(" 1M ");                    // display 1 & 10 min lits
  lcd.setCursor(0,1);
  lcd.print("10M ");

  // 1 MINUTE DISPLAY LINE . . .
  tempSum = 0;
  for (int i = 0; i <= ONE_MIN_MAX-1; i++){ // sum up 1 minute counts
    tempSum = tempSum + oneMinute[i];
  }
  temp_uSv = tempSum / uSvRate;         // calc uSv/h
  if (dispOneMin){                      // disp first so it can be overwritten by high counts
    lcd.setCursor(9, 0); 
    lcd.print("/");
    lcd.setCursor(13 - getLength(temp_uSv), 0); // right justify the uSv!
    lcd.print(temp_uSv,2);              // display 1 minute uSv
  }
  else {                                // for running counts
    lcd.setCursor(10, 0);
    lcd.print(" COUNT");
  }
  lcd.setCursor(4, 0);
  lcd.print(tempSum,DEC);               // display 1 minute CPM or running count

  // 10 MINUTE DISPLAY LINE . . .
  tempSum = 0;
  for (int i = 0; i <= TEN_MIN_MAX-1; i++){ // sum up 10 minute counts
    tempSum = tempSum + tenMinute[i];
  }
  if (dispTenMin) tempSum = tempSum / 10; // sum over 10 minutes so divide by that when CPM is displayed
  temp_uSv = tempSum / uSvRate;

  if (dispTenMin){                     // disp first so it can be overwritten by high counts
    lcd.setCursor(9, 1);
    lcd.print("/");
    lcd.setCursor(13 - getLength(temp_uSv), 1); // right justify the uSv!
    lcd.print(temp_uSv,2);              // display 1 minute uSv
  }
  else {
    lcd.setCursor(13, 1);               // right justify
    lcd.print(5*tenMinuteIndex,DEC);    // show seconds count
  }
  lcd.setCursor(4, 1);
  lcd.print(tempSum,DEC);               // display 10 minute CPM

  // give the display a little more time for fast display periods
  if (DISP_PERIOD < 5000) delay(1500);  // a delay is OK here - interrupts are still handled
}


void setAlarm(){ // RECURSIVE FUNCTION to change alarm set point when button repeatidly pushed
  long timeIn = millis();               // capture the time you got here
  while (millis() < timeIn + 2000) {    // you got 2 sec. to push button again - else done
    if (readButton(BUTTON_PIN)== LOW){  // button pushed
      if (AlarmPoint < 100) AlarmPoint = AlarmPoint +10; // inc by 10 up to 100 CPM
      else AlarmPoint = AlarmPoint +50;                  // inc by 50 over 100 CPM
      if (AlarmPoint > MAX_ALARM) AlarmPoint = 0;        // start over if max point reached - zero is off
      EEPROM.write(ALARM_SET_ADDR, AlarmPoint /10);      // store new setting in EEPROM
      lcd.setCursor(0, 1);                               // display what's going on
      clearArea (0,1,16);
      if (AlarmPoint >0){
        lcd.print(AlarmPoint); 
        lcd.print(" CPM");
      }
      else lcd.print("Alarm Off"); 
      delay(500);
      setAlarm();  // call this function recursively if button was pushed
    }
  }                // button not pushed - done use last setting for alarm point
}


void lcdBar(int counts){  // displays CPM as bargraph on 2nd line
  // Adapted from DeFex http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264215873/0
  // Had to change from 7 to 6 max blocks for display sanity
  unsigned int scaler = FULL_SCALE / 35;      // 6 char=41 "bars", scaler = counts/bar
  unsigned int cntPerBar = (counts / scaler); // amount of bars needed to display the count
  unsigned int fullBlock = (cntPerBar / 6);   // divide for full "blocks" of 6 bars 
  unsigned int prtlBlock = (cntPerBar % 6 );  // calc the remainder of bars
  if (fullBlock >6){                    // safety to prevent writing >7 blocks
    fullBlock = 6;
    prtlBlock = 0;
  }
  for (int i=0; i<fullBlock; i++){
    lcd.write(5);                       // print full blocks
  }
  lcd.write(prtlBlock);                 // print remaining bars with custom char
  for (int i=(fullBlock + 1); i<8; i++){
    lcd.print(" ");                     // blank spaces to clean up leftover
  }  
}


void logCount(unsigned long lcnt){ // unlike logging sketch, just outputs to serial
  if (millis() < logPeriodStart + LOGGING_PEROID) return; // period not over
  logCPM = float(lcnt);
  uSvLogged = logCPM / uSvRate;         // make uSV conversion

  // Print to serial in a format that might be used by Excel
//  Serial.print("  ");    
  Serial.print(logCPM,DEC);
//  Serial.print(",");                  // removed. Dose rate not required - DC
//  Serial.print(uSvLogged,4);          
//  Serial.print(","); // comma delimited
//  Serial.print(Vcc_mV/1000. ,2);        // print as volts with 2 dec. places
  Serial.println();              
  Blink(LED_PIN,2);                     // show it logged
}


void RunningCounts(unsigned long dcnt){ // Add CPM of period to 1M and 10M array
  if (altDispUsed == false)return;      // keep the totals clean until first run
  oneMinute[oneMinuteIndex] = dcnt;
  if(oneMinuteIndex >= ONE_MIN_MAX-1) {
    oneMinuteIndex = 0;
    dispOneMin = true;                  // indicate that average is available
  }
  else oneMinuteIndex++;

  tenMinute[tenMinuteIndex] = dcnt;
  if(tenMinuteIndex >= TEN_MIN_MAX-1) {
    tenMinuteIndex = 0;
    dispTenMin = true;                  // indicate that average is available
  }
  else tenMinuteIndex++;
  dcnt = 0;
}


long readVcc() { // SecretVoltmeter from TinkerIt
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


///////////////////////////////// UTILITIES ///////////////////////////////////
void clearArea (byte col, byte line, byte nspaces){
  // starting at col & line, prints n spaces then resets the cursor to the start
  lcd.setCursor(col,line);
  for (byte i=0; i<nspaces; i++){
    lcd.print(" ");
  }
  lcd.setCursor(col,line);
}


void clearDisp (){
  // The OLED display does not always reset the cursor after a clear(), so it's done here
  lcd.clear();                          // clear the screen
  lcd.setCursor(0,0);                   // reset the cursor for the poor OLED
  lcd.setCursor(0,0);                   // do it again for the OLED
}


void Blink(byte led, byte times){ // just to flash the LED
  for (byte i=0; i< times; i++){
    digitalWrite(led,HIGH);
    delay (150);
    digitalWrite(led,LOW);
    delay (100);
  }
}


int AvailRam(){ 
  int memSize = 2048;                   // if ATMega328
  byte *buf;
  while ((buf = (byte *) malloc(--memSize)) == NULL);
  free(buf);
  return memSize;
} 


byte getLength(unsigned long number){
  byte length = 0;

  for (byte i = 1; i < 10; i++){
    if (number > pow(10,i)) length = i;
    else return length +1;
  }
}


byte readButton(int buttonPin) { // reads LOW ACTIVE push buttom and debounces
  if (digitalRead(buttonPin)) return HIGH;    // still high, nothing happened, get out
  else {                                      // it's LOW - switch pushed
    delay(DEBOUNCE_MS);                       // wait for debounce period
    if (digitalRead(buttonPin)) return HIGH;  // no longer pressed
    else return LOW;                          // 'twas pressed
  }
}


void androidInput(byte flag, byte numOfValues){ // automatically called when input from Android
  androidReturn = Android.getInt();           // set global with value from slider
}

///////////////////////////////// ISR ///////////////////////////////////
void GetEvent(){   // ISR triggered for each new event (count)
  dispCnt++;
  logCnt++;
  runCnt++;
  barCnt++;
}





