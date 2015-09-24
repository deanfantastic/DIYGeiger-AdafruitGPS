// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

/* GeigerKit_Default sketch (v.8.1)                                           bHogan 6/22/12
 * This sketch was written for the DIYGeigerCounter Kit.
 * DIY Geiger invests a lot time and resources in providing this open source code, 
 * please support it by not purchasing knock-off versions of the hardware.
 * It requires the Arduino IDE rel. 1.0.0 or above to compile.
 *
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

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>              // HD44780 compatible LCDs work with this lib


// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change 
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// changed to 10 & 9 [DC]
// and make sure the switch is set to SoftSerial

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(12, 11);

Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
//Adafruit_GPS GPS(&Serial1);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

#define DEBUG         false             // if true, shows available memory
#define BUTTON_PIN       10             // button to toggle alternate display and set alarm
#define LED_PIN          13             // for debug only - flashes 5X at startup
#define ALARM_SET_ADDR    8             // address of alarm setting in EEPROM
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
unsigned long dispPeriodStart, dispPeriodLast, dispCPM; // counters for the display period . . .
unsigned long logPeriodStart, logCPM;   
unsigned long checkVccTime;             // counter for how often to check voltage
unsigned long altDispStart;             // counter for alt display refresh period
unsigned long lastButtonTime;           // counter for pressing the button to quickly
unsigned long barGraphStart;            // counter for bargraph refresh period


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


// this keeps track of whether we're using the interrupt
// off by default! [GPS]
boolean usingInterrupt = false; //<----Will this interrupt FALSE mess withthe Geiger interrupt on Pin 2?
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Start");              // print header for log
  Serial.println("Adafruit GPS library basic test!");

  // /////////////////////////////////////////////////////////////////////////////////////
  // Geiger Setup

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
  lcd.print("CPS? ");                   // display beginning "CPM"

  dispPeriodStart = millis();           // start timing display CPM
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

  lcd.createChar(0, bar_0);             // load 7 custom characters in the LCD
  lcd.createChar(1, bar_1);
  lcd.createChar(2, bar_2);
  lcd.createChar(3, bar_3);
  lcd.createChar(4, bar_4);
  lcd.createChar(5, bar_5);
  lcd.createChar(6, bar_6);

  ////////////////////////////////////////////////////////////////////////////////////////////
  // GPS setup
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}



uint32_t timer = millis();





void loop()                     // run over and over again
{
  
  //////////////////////////////////////////////////////////////////////////
  // Geiger main loop
  
  //uncomment 5 lines below for self check - you should see very close to 360 CPM
  //dispCnt++;
  //logCnt++;
  //runCnt++;
  //barCnt++;
  //delay(167);                           // 167 mS = 6 Hz `= X 60 = 360 CPM  


  if (millis() >= dispPeriodStart + DISP_PERIOD){ // DISPLAY PERIOD
    DispCounts(dispCnt, dispTime);// period is over - display counts
    dispTime = dispTime +1;
    dispPeriodStart = millis();         // reset the period time
  }

  if (millis() >= dispPeriodLast + DISP_LAST){ // DISPLAY PERIOD
    DispCounts(dispCnt, dispTime);// period is over - display counts
    Serial.print(dispCnt,DEC);
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
  
  
  //////////////////////////////////////////////////////////////////////////
  // GPS main loop
  
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}














//////////////////////////////////////////////////////////////////////////////////
// Geiger functions

void DispCounts(long dcnt, long dispTime){  // calc and display predicted CPM & uSv/h

//  dispCPM = avgCnt;    // convert to CPM - removed for next line - DC
  dispCnt = dcnt; // pass count in each second directly to LCD display
  //handle reset of sample count - sample is for 1 msec and reset. Options for reset value are:
  // "0" - throw away last average, "1" - keeps last average, "maxSamples -1" - keeps running avg.

  clearArea (0,0,16);                   // clear count area
  lcd.setCursor(0,0);
  lcd.print("Cnts |Last |");                    // display static "CPM"
  
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








//////////////////////////////////////////////////////////////////////////////
// GPS functions

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


///////////////////////////////// ISR ///////////////////////////////////
void GetEvent(){   // ISR triggered for each new event (count)
  dispCnt++;
//  lastCnt++;
}
