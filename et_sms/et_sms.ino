/***************************************************
  This was an example for the Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
THIS CODE IS STILL IN PROGRESS!

Open up the serial console on the Arduino at 115200 baud to interact with FONA

Note that if you need to set a GPRS APN, username, and password scroll down to
the commented section below at the end of the setup() function.
*/
#include <Adafruit_FONA.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "src/DS3231.h"

/* Command line printing for dev and debug */
#define OUTPUT_READABLE //uncomment to enable human readable output
#ifdef OUTPUT_READABLE
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif


const boolean HWTEST = true; // bypasses Fona commands for testing timing and hardware inputs

// TODO clean up typdef to be consistent (unsigned long vs uint32_t...)

/* Pin Defs */
#define FONA_RST 4
#define FONA_KEY 12
#define ALARM 2
#define BUTTON_HELP 3 // Panic
#define BUTTON_WORRY 5 // Worried
#define BUTTON_OK 6 // I am OK
#define LED_WAIT 9 // Orange
#define LED_OK  10 // Blue
#define LED_ACK 11 // Green
#define LEDPAT_OK 1000 
#define LEDPAT_LONG 3000
#define LEDPAT_SHORT 500
#define FONA_WAKE 2000
#define FONA_REST 3000

// this is a large buffer for replies
char replybuffer[255];
const uint8_t DEBOUNCE = 20;  // ms for software switch debounce


#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
  // For UNO and others without hardware serial,
  // we default to using software serial. If you want to use hardware serial
  // (because softserial isnt supported) comment out the following three lines 
  // and uncomment the HardwareSerial line
  #include <SoftwareSerial.h>
  
  #define FONA_RX 5
  #define FONA_TX 6
  SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
  SoftwareSerial *fonaSerial = &fonaSS;

#else
  // On Leonardo/M0/etc, others with hardware serial, use hardware serial!
  HardwareSerial *fonaSerial = &Serial1;
#endif

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

/* Overload fn prototype */
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

// Hardcode module used
const uint8_t type = FONA800H;

// RTC for alarm functionality.  Currently using Chronodot v2.1 w/ DS3231 IC
DS3231 myRTC;
RTCDateTime dt;

// Interrupt flags for hw interrupt
volatile boolean isAlarm = false;
volatile boolean panic = false;

/* ISR for RTC alarm */
void alarmFunction() {
  isAlarm = true;
  detachInterrupt(digitalPinToInterrupt(ALARM));
  // Clear INT flag
  EIFR = 0b00000001;
}

/* ISR for button press */ 
void buttonPress() {
  panic = true; // not really though, we got this
  detachInterrupt(digitalPinToInterrupt(BUTTON_HELP));
  // Clear INT flag
  EIFR = 0b00000001;
}


void enterSleep(void)
{
  // Put radio to sleep using KEY pin on 800H module
  digitalWrite(FONA_KEY, LOW);
  delay(2000);
  digitalWrite(FONA_KEY, HIGH);
  delay(3000);

  // set SFRs for sleep, see ATMEGA328p manual for dets
  sleep_enable(); 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  
  // Attach RTC alarm interrput. In Arduino UNO connect DS3231 INT/SQW to Arduino Pin 2
  attachInterrupt(digitalPinToInterrupt(ALARM), alarmFunction, FALLING);
  // Attach button press interrupt.  In arduino, connect grounded button to pin 3.
  attachInterrupt(digitalPinToInterrupt(BUTTON_HELP), buttonPress, FALLING);
  delay(50);

  // CPU stops executing HERE
  sleep_cpu();
  
  // Wake CPU, make sure all clocks are stable
  sleep_disable();
  delay(10);

  // TODO build subroutine to start blinking the LED to let user know we got their input from button
  // Wake up FONA
  unsigned long tick = millis();
  uint8_t iter = 1;
  
  digitalWrite(FONA_KEY, LOW);
  boolean state = digitalRead(LED_OK);
  while(millis() < tick + FONA_WAKE) {
    if(millis() > tick + (iter*LEDPAT_OK)) {
      digitalWrite(LED_OK, state);
      state = !state;
      iter += 1;
    }
  }
  digitalWrite(FONA_KEY, HIGH);
  tick = millis();
  iter = 1;
  while(millis() < tick + FONA_REST) {
    if(millis() > tick + (iter*LEDPAT_OK)) {
      digitalWrite(LED_OK, state);
      state = !state;
      iter += 1;
    }
  }
  digitalWrite(LED_OK, LOW);

  if(!HWTEST){
    if (! fona.begin(*fonaSerial)) {
      DEBUG_PRINTLN(F("Couldn't find FONA"));
      while(! fona.begin(*fonaSerial)) {
        DEBUG_PRINTLN("Retrying FONA serial boot in 10 seconds...");
        delay(10000);
      }
    }
  }
  
}


void setup() {

  Serial.begin(115200);
  
   // Initialize DS3231 RTC
  DEBUG_PRINTLN("Initialize DS3231 RTC...");;

  // Begins I2C bus for RTC comm
  myRTC.begin();
  
  // Set control register to enable interrupt (active LOW) on DS3231
  myRTC.enableAlarmInt();
  
  // INT/SQW pin on RTC is active low; use hw interrupt 
  pinMode(ALARM, INPUT_PULLUP);
  // Tie momentary button to ground s.t. pulls pin low when pressed.
  pinMode(BUTTON_HELP, INPUT_PULLUP);

  // indicator LEDs
  pinMode(LED_ACK, OUTPUT);
  pinMode(LED_OK, OUTPUT);
  pinMode(LED_WAIT, OUTPUT);
  digitalWrite(LED_ACK, LOW);
  digitalWrite(LED_OK, LOW);
  digitalWrite(LED_WAIT, LOW);
  
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, HIGH);

    
  myRTC.armAlarm1(true);
  // Manual (Year, Month, Day, Hour, Minute, Second)
  myRTC.setDateTime(22, 5, 10, 10, 30, 0);
  // setAlarm1(Date or Day, Hour, Minute, Second, Mode, Armed = true)
  myRTC.setAlarm1(2, 10, 31, 0, DS3231_MATCH_DY_H_M_S, true);

  DEBUG_PRINTLN(F("Initializing....(May take 3 seconds)"));

  if(!HWTEST){
    fonaSerial->begin(4800);
    // TODO build status LED blink during bootup
    if (! fona.begin(*fonaSerial)) {
      DEBUG_PRINTLN(F("Couldn't find FONA"));
      while(! fona.begin(*fonaSerial)) {
        DEBUG_PRINTLN("Retrying FONA serial boot in 10 seconds...");
        delay(10000);
      }
    }
    DEBUG_PRINTLN(F("FONA is OK"));
    
    // Print module IMEI number.
    char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
    uint8_t imeiLen = fona.getIMEI(imei);
    if (imeiLen > 0) {
      DEBUG_PRINT("Module IMEI: "); DEBUG_PRINTLN(imei);
    }
  
    // Provider of the sim defines APN, username,
    // and password values.  Username and password are optional and
    // can be removed, but APN is required.
    fona.setGPRSNetworkSettings(F("m2mglobal"), F(""), F(""));
    delay(10);
  }
  
  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

  DEBUG_PRINTLN("\nBoot up complete.\n");
}


void printMenu(void) {
  DEBUG_PRINTLN(F("-------------------------------------"));
  DEBUG_PRINTLN(F("[?] Print this menu"));
  DEBUG_PRINTLN(F("[a] read the ADC 2.8V max (FONA800 & 808)"));
  DEBUG_PRINTLN(F("[b] read the Battery V and % charged"));
  DEBUG_PRINTLN(F("[C] read the SIM CCID"));
  DEBUG_PRINTLN(F("[U] Unlock SIM with PIN code"));
  DEBUG_PRINTLN(F("[i] read RSSI"));
  DEBUG_PRINTLN(F("[n] get Network status"));
  DEBUG_PRINTLN(F("[v] set audio Volume"));
  DEBUG_PRINTLN(F("[V] get Volume"));
  DEBUG_PRINTLN(F("[H] set Headphone audio (FONA800 & 808)"));
  DEBUG_PRINTLN(F("[e] set External audio (FONA800 & 808)"));
  DEBUG_PRINTLN(F("[T] play audio Tone"));
  DEBUG_PRINTLN(F("[P] PWM/Buzzer out (FONA800 & 808)"));

  // FM (SIM800 only!)
  DEBUG_PRINTLN(F("[f] tune FM radio (FONA800)"));
  DEBUG_PRINTLN(F("[F] turn off FM (FONA800)"));
  DEBUG_PRINTLN(F("[m] set FM volume (FONA800)"));
  DEBUG_PRINTLN(F("[M] get FM volume (FONA800)"));
  DEBUG_PRINTLN(F("[q] get FM station signal level (FONA800)"));

  // Phone
  DEBUG_PRINTLN(F("[c] make phone Call"));
  DEBUG_PRINTLN(F("[A] get call status"));
  DEBUG_PRINTLN(F("[h] Hang up phone"));
  DEBUG_PRINTLN(F("[p] Pick up phone"));

  // SMS
  DEBUG_PRINTLN(F("[N] Number of SMSs"));
  DEBUG_PRINTLN(F("[r] Read SMS #"));
  DEBUG_PRINTLN(F("[R] Read All SMS"));
  DEBUG_PRINTLN(F("[d] Delete SMS #"));
  DEBUG_PRINTLN(F("[s] Send SMS"));
  DEBUG_PRINTLN(F("[u] Send USSD"));
  
  // Time
  DEBUG_PRINTLN(F("[y] Enable network time sync (FONA 800 & 808)"));
  DEBUG_PRINTLN(F("[Y] Enable NTP time sync (GPRS FONA 800 & 808)"));
  DEBUG_PRINTLN(F("[t] Get network time"));

  // GPRS
  DEBUG_PRINTLN(F("[G] Enable GPRS"));
  DEBUG_PRINTLN(F("[g] Disable GPRS"));
  DEBUG_PRINTLN(F("[l] Query GSMLOC (GPRS)"));
  DEBUG_PRINTLN(F("[w] Read webpage (GPRS)"));
  DEBUG_PRINTLN(F("[W] Post to website (GPRS)"));
}


void debounce_switch(uint8_t pin) {
  boolean state;
  boolean prior;
  
  unsigned long counter = millis();
  prior = digitalRead(pin);
  while( millis() < counter + DEBOUNCE ) {
	delay(5);
	state = digitalRead(pin);
  	if( state != prior ) {
  	  counter = millis();
  	  prior = state;
  	}
  }

}


void loop() {
  
  while (!Serial.available() ) {
    // Power saving sleep. Will wake on external alarm
    DEBUG_PRINTLN("Sleeping...");
    enterSleep();
    
    
    // Check condition to reconnect to the network
    if(isAlarm){      
      DEBUG_PRINTLN("Alarm has been triggered...");
      char sendto[] = "18477781009";
      char message[] = "Standard connection test successful. Unit will return to standby.";
      isAlarm = false;
      // TODO try again?
      if(!HWTEST){
        if (!fona.sendSMS(sendto, message)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("Sent!"));
        }
      }
      // re-enable interrupt
      attachInterrupt(digitalPinToInterrupt(ALARM), alarmFunction, FALLING);
    }
    
    if(panic){
      // wait for switch to settle (default=20ms)
      debounce_switch(BUTTON_HELP);
      char sendto[] = "18477781009";
      char message[140];
      // Get which button was pressed.  Worry and ok are tied to pull down other pins when pressed.
      if(!digitalRead(BUTTON_WORRY)){
        DEBUG_PRINTLN("Worry button has been triggered...");  
        
        strcpy(message, "Dorothy is worried. Reply OK to light the ACK indicator.\0");
      } else if(!digitalRead(BUTTON_OK)){
        DEBUG_PRINTLN("OK button has been triggered...");  
        
        strcpy(message, "Dorothy is OK. Reply OK to light the ACK indicator.\0");
      } else {
        DEBUG_PRINTLN("Help button has been triggered...");  
        
        strcpy(message, "Dorothy needs some help. Reply OK to light the ACK indicator.\0");
      }
  
      panic = false; // ohthankgod

      if(!HWTEST){
        // TODO try again?
        if (!fona.sendSMS(sendto, message)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("Sent!"));
        }
      }
      // re-enable interrupt just in case
      attachInterrupt(digitalPinToInterrupt(BUTTON_HELP), buttonPress, FALLING);
    }
   
    if(!HWTEST){
      if (fona.available()) {
        Serial.write(fona.read());
      }
    }
    
  }


  // TODO Tidy this up and fix the menu as well.
  char command = Serial.read();
  DEBUG_PRINTLN(command);

  if(!HWTEST){
  switch (command) {
    case '?': {
        printMenu();
        break;
      }

    case 'a': {
        // read the ADC
        uint16_t adc;
        if (! fona.getADCVoltage(&adc)) {
          DEBUG_PRINTLN(F("Failed to read ADC"));
        } else {
          DEBUG_PRINT(F("ADC = ")); DEBUG_PRINT(adc); DEBUG_PRINTLN(F(" mV"));
        }
        break;
      }

    case 'b': {
        // read the battery voltage and percentage
        uint16_t vbat;
        if (! fona.getBattVoltage(&vbat)) {
          DEBUG_PRINTLN(F("Failed to read Batt"));
        } else {
          DEBUG_PRINT(F("VBat = ")); DEBUG_PRINT(vbat); DEBUG_PRINTLN(F(" mV"));
        }


        if (! fona.getBattPercent(&vbat)) {
          DEBUG_PRINTLN(F("Failed to read Batt"));
        } else {
          DEBUG_PRINT(F("VPct = ")); DEBUG_PRINT(vbat); DEBUG_PRINTLN(F("%"));
        }

        break;
      }

    case 'U': {
        // Unlock the SIM with a PIN code
        char PIN[5];
        flushSerial();
        DEBUG_PRINTLN(F("Enter 4-digit PIN"));
        readline(PIN, 3);
        DEBUG_PRINTLN(PIN);
        DEBUG_PRINTLN(F("Unlocking SIM card: "));
        if (! fona.unlockSIM(PIN)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        break;
      }

    case 'C': {
        // read the CCID
        fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
        DEBUG_PRINT(F("SIM CCID = ")); DEBUG_PRINTLN(replybuffer);
        break;
      }

    case 'i': {
        // read the RSSI
        uint8_t n = fona.getRSSI();
        int8_t r;

        DEBUG_PRINT(F("RSSI = ")); DEBUG_PRINT(n); DEBUG_PRINTLN(": ");
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
        DEBUG_PRINT(r); DEBUG_PRINTLN(F(" dBm"));

        break;
      }

    case 'n': {
        // read the network/cellular status
        uint8_t n = fona.getNetworkStatus();
        DEBUG_PRINT(F("Network status ")); DEBUG_PRINT(n); DEBUG_PRINTLN(F(": "));
        if (n == 0) DEBUG_PRINTLN(F("Not registered"));
        if (n == 1) DEBUG_PRINTLN(F("Registered (home)"));
        if (n == 2) DEBUG_PRINTLN(F("Not registered (searching)"));
        if (n == 3) DEBUG_PRINTLN(F("Denied"));
        if (n == 4) DEBUG_PRINTLN(F("Unknown"));
        if (n == 5) DEBUG_PRINTLN(F("Registered roaming"));
        break;
      }

    /*** Audio ***/
    case 'v': {
        // set volume
        flushSerial();
        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          DEBUG_PRINTLN(F("Set Vol [0-8] "));
        } else {
          DEBUG_PRINTLN(F("Set Vol % [0-100] "));
        }
        uint8_t vol = readnumber();
        DEBUG_PRINTLN();
        if (! fona.setVolume(vol)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        break;
      }

    case 'V': {
        uint8_t v = fona.getVolume();
        DEBUG_PRINTLN(v);
        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          DEBUG_PRINTLN(" / 8");
        } else {
          DEBUG_PRINTLN("%");
        }
        break;
      }

    case 'H': {
        // Set Headphone output
        if (! fona.setAudio(FONA_HEADSETAUDIO)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        fona.setMicVolume(FONA_HEADSETAUDIO, 15);
        break;
      }
    case 'e': {
        // Set External output
        if (! fona.setAudio(FONA_EXTAUDIO)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }

        fona.setMicVolume(FONA_EXTAUDIO, 10);
        break;
      }

    case 'T': {
        // play tone
        flushSerial();
        DEBUG_PRINTLN(F("Play tone #"));
        uint8_t kittone = readnumber();
        DEBUG_PRINTLN("");
        // play for 1 second (1000 ms)
        if (! fona.playToolkitTone(kittone, 1000)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        break;
      }

    /*** FM Radio ***/

    case 'f': {
        // get freq
        flushSerial();
        DEBUG_PRINTLN(F("FM Freq (eg 1011 == 101.1 MHz): "));
        uint16_t station = readnumber();
        DEBUG_PRINTLN("");
        // FM radio ON using headset
        if (fona.FMradio(true, FONA_HEADSETAUDIO)) {
          DEBUG_PRINTLN(F("Opened"));
        }
        if (! fona.tuneFMradio(station)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("Tuned"));
        }
        break;
      }
    case 'F': {
        // FM radio off
        if (! fona.FMradio(false)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        break;
      }
    case 'm': {
        // Set FM volume.
        flushSerial();
        DEBUG_PRINTLN(F("Set FM Vol [0-6]:"));
        uint8_t vol = readnumber();
        DEBUG_PRINTLN("");
        if (!fona.setFMVolume(vol)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        break;
      }
    case 'M': {
        // Get FM volume.
        uint8_t fmvol = fona.getFMVolume();
        if (fmvol < 0) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINT(F("FM volume: "));
          DEBUG_PRINTLN(fmvol, DEC);
        }
        break;
      }
    case 'q': {
        // Get FM station signal level (in decibels).
        flushSerial();
        DEBUG_PRINTLN(F("FM Freq (eg 1011 == 101.1 MHz): "));
        uint16_t station = readnumber();
        DEBUG_PRINTLN("");
        int8_t level = fona.getFMSignalLevel(station);
        if (level < 0) {
          DEBUG_PRINTLN(F("Failed! Make sure FM radio is on (tuned to station)."));
        } else {
          DEBUG_PRINT(F("Signal level (dB): "));
          DEBUG_PRINTLN(level, DEC);
        }
        break;
      }

    /*** PWM ***/

    case 'P': {
        // PWM Buzzer output @ 2KHz max
        flushSerial();
        DEBUG_PRINTLN(F("PWM Freq, 0 = Off, (1-2000): "));
        uint16_t freq = readnumber();
        DEBUG_PRINTLN("");
        if (! fona.setPWM(freq)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        break;
      }

    /*** Call ***/
    case 'c': {
        // call a phone!
        char number[30];
        flushSerial();
        DEBUG_PRINTLN(F("Call #"));
        readline(number, 30);
        DEBUG_PRINTLN("");
        DEBUG_PRINT(F("Calling ")); DEBUG_PRINTLN(number);
        if (!fona.callPhone(number)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("Sent!"));
        }

        break;
      }
    case 'A': {
        // get call status
        int8_t callstat = fona.getCallStatus();
        switch (callstat) {
          case 0: DEBUG_PRINTLN(F("Ready")); break;
          case 1: DEBUG_PRINTLN(F("Could not get status")); break;
          case 3: DEBUG_PRINTLN(F("Ringing (incoming)")); break;
          case 4: DEBUG_PRINTLN(F("Ringing/in progress (outgoing)")); break;
          default: DEBUG_PRINTLN(F("Unknown")); break;
        }
        break;
      }
      
    case 'h': {
        // hang up!
        if (! fona.hangUp()) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        break;
      }

    case 'p': {
        // pick up!
        if (! fona.pickUp()) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("OK!"));
        }
        break;
      }

    /*** SMS ***/

    case 'N': {
        // read the number of SMS's!
        int8_t smsnum = fona.getNumSMS();
        if (smsnum < 0) {
          DEBUG_PRINTLN(F("Could not read # SMS"));
        } else {
          DEBUG_PRINT(smsnum);
          DEBUG_PRINTLN(F(" SMS's on SIM card!"));
        }
        break;
      }
    case 'r': {
        // read an SMS
        flushSerial();
        DEBUG_PRINTLN(F("Read #"));
        uint8_t smsn = readnumber();
        DEBUG_PRINT(F("\n\rReading SMS #")); DEBUG_PRINTLN(smsn);

        // Retrieve SMS sender address/phone number.
        if (! fona.getSMSSender(smsn, replybuffer, 250)) {
          DEBUG_PRINTLN("Failed!");
          break;
        }
        DEBUG_PRINT(F("FROM: ")); DEBUG_PRINTLN(replybuffer);

        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          DEBUG_PRINTLN("Failed!");
          break;
        }
        DEBUG_PRINT(F("***** SMS #")); DEBUG_PRINT(smsn);
        DEBUG_PRINT(" ("); DEBUG_PRINT(smslen); DEBUG_PRINT(F(") bytes *****"));
        DEBUG_PRINT(replybuffer);
        DEBUG_PRINTLN(F("*****"));

        break;
      }
    case 'R': {
        // read all SMS
        int8_t smsnum = fona.getNumSMS();
        uint16_t smslen;
        int8_t smsn;

        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          smsn = 0; // zero indexed
          smsnum--;
        } else {
          smsn = 1;  // 1 indexed
        }

        for ( ; smsn <= smsnum; smsn++) {
          DEBUG_PRINT(F("\n\rReading SMS #")); DEBUG_PRINTLN(smsn);
          if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
            DEBUG_PRINTLN(F("Failed!"));
            break;
          }
          // if the length is zero, its a special case where the index number is higher
          // so increase the max we'll look at!
          if (smslen == 0) {
            DEBUG_PRINTLN(F("[empty slot]"));
            smsnum++;
            continue;
          }

          DEBUG_PRINT(F("***** SMS #")); DEBUG_PRINT(smsn);
          DEBUG_PRINT(" ("); DEBUG_PRINT(smslen); DEBUG_PRINT(F(") bytes *****"));
          DEBUG_PRINT(replybuffer);
          DEBUG_PRINTLN(F("*****"));
        }
        break;
      }

    case 'd': {
        // delete an SMS
        flushSerial();
        DEBUG_PRINTLN(F("Delete #"));
        uint8_t smsn = readnumber();

        DEBUG_PRINT(F("\n\rDeleting SMS #")); DEBUG_PRINTLN(smsn);
        if (fona.deleteSMS(smsn)) {
          DEBUG_PRINTLN(F("OK!"));
        } else {
          DEBUG_PRINTLN(F("Couldn't delete"));
        }
        break;
      }

    case 's': {
        // send an SMS!
//        char sendto[21], message[141];
        flushSerial();
//        DEBUG_PRINT(F("Send to #"));
//        readline(sendto, 20);
//        DEBUG_PRINTLN(sendto);
//        DEBUG_PRINT(F("Type out one-line message (140 char): "));
//        readline(message, 140);
//        DEBUG_PRINTLN(message);
        char sendto[] = "18477781009";
        char message[] = "Hello, this is a test.";
        if (!fona.sendSMS(sendto, message)) {
          DEBUG_PRINTLN(F("Failed"));
        } else {
          DEBUG_PRINTLN(F("Sent!"));
        }

        break;
      }

    case 'u': {
      // send a USSD!
      char message[141];
      flushSerial();
      DEBUG_PRINT(F("Type out one-line message (140 char): "));
      readline(message, 140);
      DEBUG_PRINTLN(message);

      uint16_t ussdlen;
      if (!fona.sendUSSD(message, replybuffer, 250, &ussdlen)) { // pass in buffer and max len!
        DEBUG_PRINTLN(F("Failed"));
      } else {
        DEBUG_PRINTLN(F("Sent!"));
        DEBUG_PRINT(F("***** USSD Reply"));
        DEBUG_PRINT(" ("); DEBUG_PRINT(ussdlen); DEBUG_PRINT(F(") bytes *****"));
        DEBUG_PRINT(replybuffer);
        DEBUG_PRINTLN(F("*****"));
      }
    }

    /*** Time ***/

    case 'y': {
        // enable network time sync
        if (!fona.enableNetworkTimeSync(true))
          DEBUG_PRINTLN(F("Failed to enable"));
        break;
      }

    case 'Y': {
        // enable NTP time sync
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
          DEBUG_PRINTLN(F("Failed to enable"));
        break;
      }

    case 't': {
        // read the time
        char buffer[23];

        fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
        DEBUG_PRINT(F("Time = ")); DEBUG_PRINTLN(buffer);
        break;
      }


   

    /*********************************** GPRS */

    case 'g': {
        // turn GPRS off
        if (!fona.enableGPRS(false))
          DEBUG_PRINTLN(F("Failed to turn off"));
        break;
      }
    case 'G': {
        // turn GPRS on
        if (!fona.enableGPRS(true))
          DEBUG_PRINTLN(F("Failed to turn on"));
        break;
      }
    case 'l': {
        // check for GSMLOC (requires GPRS)
        uint16_t returncode;

        if (!fona.getGSMLoc(&returncode, replybuffer, 250))
          DEBUG_PRINTLN(F("Failed!"));
        if (returncode == 0) {
          DEBUG_PRINTLN(replybuffer);
        } else {
          DEBUG_PRINT(F("Fail code #")); DEBUG_PRINTLN(returncode);
        }

        break;
      }
    case 'w': {
        // read website URL
        uint16_t statuscode;
        int16_t length;
        char url[80];

        flushSerial();
        DEBUG_PRINTLN(F("NOTE: in beta! Use small webpages to read!"));
        DEBUG_PRINTLN(F("URL to read (e.g. wifitest.adafruit.com/testwifi/index.html):"));
        DEBUG_PRINT(F("http://")); readline(url, 79);
        DEBUG_PRINTLN(url);

        DEBUG_PRINT(F("****"));
        if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
          DEBUG_PRINTLN("Failed!");
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

            // Serial.write is too slow, we'll write directly to Serial register!
        #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
        #else
            Serial.write(c);
        #endif
            length--;
            if (! length) break;
          }
        }
        DEBUG_PRINTLN(F("\n****"));
        fona.HTTP_GET_end();
        break;
      }

    case 'W': {
        // Post data to website
        uint16_t statuscode;
        int16_t length;
        char url[80];
        char data[80];

        flushSerial();
        DEBUG_PRINTLN(F("NOTE: in beta! Use simple websites to post!"));
        DEBUG_PRINTLN(F("URL to post (e.g. httpbin.org/post):"));
        DEBUG_PRINT(F("http://")); readline(url, 79);
        DEBUG_PRINTLN(url);
        DEBUG_PRINTLN(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
        readline(data, 79);
        DEBUG_PRINTLN(data);

        DEBUG_PRINTLN(F("****"));
        if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
          DEBUG_PRINTLN("Failed!");
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

        #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
        #else
            Serial.write(c);
        #endif

            length--;
            if (! length) break;
          }
        }
        DEBUG_PRINTLN(F("\n****"));
        fona.HTTP_POST_end();
        break;
      }
    /*****************************************/

    case 'S': {
        DEBUG_PRINTLN(F("Creating SERIAL TUBE"));
        while (1) {
          while (Serial.available()) {
            delay(1);
            fona.write(Serial.read());
          }
          if (fona.available()) {
            Serial.write(fona.read());
          }
        }
        break;
      }

    default: {
        DEBUG_PRINTLN(F("Unknown command"));
        printMenu();
        break;
      }
  }
  // flush input
  flushSerial();
  while (fona.available()) {
    Serial.write(fona.read());
  }

  }// end of if HWTEST

}// end of loop()



void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //DEBUG_PRINT(c);
  }
  DEBUG_PRINT(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    DEBUG_PRINT(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //DEBUG_PRINTLN(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //DEBUG_PRINT(c, HEX); DEBUG_PRINT("#"); DEBUG_PRINTLN(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //DEBUG_PRINTLN(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}
