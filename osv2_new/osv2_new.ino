// decode Oregon Scientific sensors, V2 protocol. V3.2 SJR 12/8/2018
// checked 11/2/2026, working OK with THN132N sensor and RXB6 or YXR15 receivers
// based on code from (and others)
// Version 1.0 - Hubert Mickael <mickael@winlux.fr> (https://github.com/Mickaelh51)
// see also https://jeelabs.net/projects/cafe/wiki/Decoding_the_Oregon_Scientific_V2_protocol
// "Simple C" version of lib
// added checksum test for THN132N and THRN122N sensors.
// For checksum test to work, above sensors must be identified as or set to channels 2 and 1, respectively

#include "OSV2_lib.h"
#define RECEIVER_PIN 2

// *** receiver interrupt handler ***
volatile unsigned int transition_width;
void ext_int_1(void)
{
  static unsigned long last;
  // determine the pulse length in microseconds, for either polarity
  transition_width = micros() - last;
  last += transition_width;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Setup received data
  Serial.println("Decoding OSV2 sensors THN132N, THRN122N V3.2");
  attachInterrupt(digitalPinToInterrupt(RECEIVER_PIN), ext_int_1, CHANGE);
  Serial.println("Setup done");
}
unsigned long last_print[4] = {0,0,0,0}; //eliminate duplicate records, for each channel
void loop () {

  // retrieve current pulse width
  cli();
  unsigned int p = transition_width;
  transition_width = 0;
  sei();
  if (p != 0)
  {
    if (decode(p)) //done decoding?
    {
        unsigned char ch = channel();  //0 to 3?
        unsigned char v = valid_message(ch); //good checksum?
        if (!v) dump_record();
        if ( v && (millis() - last_print[ch] > 30000UL)) { //once per minute or so
        float Tc = temperature(); 
        unsigned char batt = battery();
        unsigned char tr = trend();
        Serial.print("T ");
        Serial.print(9*Tc/5 + 32.0,1);
        Serial.print(" F ch ");
        Serial.print(ch);
        Serial.print(" batt ");
        Serial.print(batt);
        Serial.print(" trend ");
        Serial.println(tr);
//        dump_record();  //temp stored in bytes 5 and upper nibble of 4, e.g. Hex "2018" = 18.2 Celcius
        last_print[ch] = millis();
      }
    resetDecoder();
    }
  }
}
