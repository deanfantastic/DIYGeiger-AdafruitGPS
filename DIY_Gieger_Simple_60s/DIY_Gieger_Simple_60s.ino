/* GeigerKit_Default sketch (v.8.1)                                           bHogan 6/22/12
 * This sketch was written for the DIYGeigerCounter Kit.
 * DIY Geiger invests a lot time and resources in providing this open source code, 
 * please support it by not purchasing knock-off versions of the hardware.
 * It requires the Arduino IDE rel. 1.0.0 or above to compile.
 * DIY Geiger uses Arduino Uno board....apparently.

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
 */

#include <SoftwareSerial.h>
#include <LiquidCrystal.h>              // HD44780 compatible LCDs work with this lib

#define DEBUG         false             // if true, shows available memory
#define BUTTON_PIN       10             // button to toggle alternate display and set alarm
#define LED_PIN          13             // for debug only - flashes 5X at startup
#define ALARM_SET_ADDR    8             // address of alarm setting in EEPROM


// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(12, 11);

// mS between writes to display - counts/period are averaged to CPM
#define DISP_PERIOD      1000.0         // (1 sec) mS sample & display
#define DISP_LAST        60000.0          // (1 sec) ms last count) - DEBUG

#define LOW_VCC            4200 //mV    // if Vcc < LOW_VCC give low voltage warning
#define DEBOUNCE_MS          50         // buttom debounce period in mS

boolean lowVcc = false;                 // true when Vcc < LOW_VCC
boolean altDispUsed = false;            // used to start counts from zero
boolean altDispOn = false;              // true when SW_1 on, and in continous count mode
boolean dispOneMin = false;             // true if enough time passed to begin displaying counts


int Vcc_mV;                             // mV of Vcc from last check 


volatile unsigned long dispCnt, lastCnt, dispTime; // inc by ISR
volatile unsigned long runCnt, barCnt;
long runCountStart;
unsigned long tempSum;
unsigned long dispPeriodStart, dispPeriodLast; // counters for the display period . . .
unsigned long logPeriodStart, logCPM;   
unsigned long checkVccTime;             // counter for how often to check voltage
unsigned long altDispStart;             // counter for alt display refresh period
unsigned long lastButtonTime;           // counter for pressing the button to quickly
unsigned long barGraphStart;            // counter for bargraph refresh period
float DispCountRate;            // count rate (cps) for serial output

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


void setup(){

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Start");              // print header for log
 
  attachInterrupt(0,GetEvent,FALLING);  // Geiger event on pin 2 triggers interrupt
  pinMode(LED_PIN,OUTPUT);              // setup LED pin
  pinMode(BUTTON_PIN,INPUT);            // setup menu button
  digitalWrite(BUTTON_PIN, HIGH);
  Blink(LED_PIN,4);                     // show it's alive
  lcd.begin(16,2);                      // cols, rows of display (8x2, 16x2, etc.)

  lcd.setCursor(0,0);

  clearDisp();                          // clear the screen
  lcd.print("60sec count");             // display a simple banner
  lcd.setCursor(0,1);                   // set cursor on line 2
  lcd.print("Ver. 1.0");                // display the version
  delay (1500);                         // leave the banner up for a bit
  clearDisp();                          // clear the screen

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

  clearDisp();                          // clear the screen

  dispPeriodStart = millis();           // start timing display
  dispPeriodLast = millis();
  logPeriodStart = dispPeriodStart;     // start logging timer
  checkVccTime = dispPeriodStart;       // start Vcc timer
  runCountStart = dispPeriodStart;      // start alternate timer
  barGraphStart = dispPeriodStart;      // start bargraph timer
  dispCnt = 0;                          // start with fresh totals
  lastCnt= 99999;
  dispTime = 0;
  runCnt = 0;
  barCnt = 0;
  DispCountRate = 0;

  lcd.createChar(0, bar_0);             // load 7 custom characters in the LCD
  lcd.createChar(1, bar_1);
  lcd.createChar(2, bar_2);
  lcd.createChar(3, bar_3);
  lcd.createChar(4, bar_4);
  lcd.createChar(5, bar_5);
  lcd.createChar(6, bar_6);

}


void loop(){

  if (millis() >= dispPeriodStart + DISP_PERIOD){ // DISPLAY PERIOD
    DispCounts(dispCnt, dispTime);// period is over - display counts
    dispTime = dispTime +1;
    dispPeriodStart = millis();         // reset the period time
  }

  if (millis() >= dispPeriodLast + DISP_LAST){ // DISPLAY PERIOD
    DispCounts(dispCnt, dispTime);// period is over - display counts
    Serial.println();
    Serial.print("Final count: ");
    Serial.print(dispCnt,DEC);
    Serial.print(" counts");
    Serial.println();
    Serial.println();  
    lastCnt = dispCnt;
    dispCnt = 0;                        // reset counter
    dispTime = 0;
    dispPeriodLast = millis();         // reset the period time
  }

  if (millis() >= checkVccTime + 1000){ // timer for check battery (every 6 sec.)
    checkVccTime = millis();            // reset timer
    Vcc_mV = readVcc();
    if (Vcc_mV <= LOW_VCC) lowVcc = true; // check if Vcc is low 
    else lowVcc = false;
  }
}


void DispCounts(long dcnt, long dispTime){  // calc and display count stats

  dispCnt = dcnt; // pass count in each second directly to LCD display
  //handle reset of sample count - sample is for 1 msec and reset. Options for reset value are:
  // "0" - throw away last average, "1" - keeps last average, "maxSamples -1" - keeps running avg.

  clearArea (0,0,16);                   // clear count area
  lcd.setCursor(0,0);
  lcd.print("Cnts |Last |");                    
    
  lcd.setCursor(12,0);
  lcd.print(dispTime);
  lcd.print("s");
  
  lcd.setCursor(15,0);
  lcd.write(dispTime/10);
  
  clearArea (0,1,16);                   // clear line 2
  lcd.setCursor(0,1);
  lcd.print(dispCnt);                   // display CPM on line 1
  lcd.setCursor(5,1);
  lcd.print("|");
  lcd.print(lastCnt);                   // display CPM on line 1
  lcd.setCursor(11,1);
  lcd.print("|");
  lcd.print(Vcc_mV/1000. ,1);           // display as volts with 2 dec. places
  lcd.print("v");

// Send data to serial
  Serial.print("Current count: ");
  Serial.print(dispCnt);
  Serial.print(" in ");
  Serial.print(dispTime);
  Serial.print(" seconds (");
  DispCountRate = float(dispCnt)/float(dispTime);
  
  // don't calcualte for zeroth second reading
  if (dispTime == 0){
    Serial.print("---");
  }else{
    Serial.print(DispCountRate, 2);
  }
  Serial.print(" cps)");
  Serial.println();

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




byte readButton(int buttonPin) { // reads LOW ACTIVE push buttom and debounces
  if (digitalRead(buttonPin)) return HIGH;    // still high, nothing happened, get out
  else {                                      // it's LOW - switch pushed
    delay(DEBOUNCE_MS);                       // wait for debounce period
    if (digitalRead(buttonPin)) return HIGH;  // no longer pressed
    else return LOW;                          // 'twas pressed
  }
}



///////////////////////////////// ISR ///////////////////////////////////
void GetEvent(){   // ISR triggered for each new event (count)
  dispCnt++;
//  lastCnt++;
}





