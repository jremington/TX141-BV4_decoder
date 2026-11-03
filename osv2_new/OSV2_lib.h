// decode Oregon Scientific sensors, V2 protocol.
// based on code from (et al.)
// Version 1.0 - Hubert Mickael <mickael@winlux.fr> (https://github.com/Mickaelh51)
// see also https://jeelabs.net/projects/cafe/wiki/Decoding_the_Oregon_Scientific_V2_protocol// "Simple C" version
// added simple checksum test for THN132N and THRN122N sensors
// "Simple C" version SJR 12/7/2018

#ifndef OSV2_libH
#define OSV2_libH
//globals

byte total_edges, bits, flip, state, index, data[12];  //at most 10 bytes expected
enum { START, WAIT, NEXT_BIT, DONE };  //state machine for decoding


void resetDecoder () {
  total_edges = bits = index = flip = 0;
  state = START;
}

// store every other bit
void putBit (char value) {
  if ( (total_edges & 0x01) == 0) //store every other transition
  {
    data[index] = (data[index] >> 1) | (value ? 0x80 : 00);
    if (++bits >= 8) bits = 0;  //for eventual padding at end of transmission
  }
  total_edges++;
  index = total_edges >> 4;  //byte index to data
  if (index >= sizeof data) { //overflow data array?
    //Serial.println("sizeof data");
    resetDecoder();
    return;
  }
  state = NEXT_BIT;
}

// done, pad last remaining byte with zeros, if needed
void done () {
  if (bits > 0) {
    //    Serial.print(" padding ");  //debug
    //    Serial.println(bits);
    while (bits) {
      data[index] = (data[index] >> 1);
      if (++bits >= 8) bits = 0;
    }
  }
  state = DONE;
}

// decode Manchester bits. Input: time in microseconds from last transition
// output 1 if message is complete, 0 otherwise

char decode (unsigned int width) {

  if (200 <= width && width < 1250) { //avg widths ~450 and ~950 microseconds
    //valid data pulse
    byte long_pulse = (width >= 730); // boolean

    switch (state) {
      case DONE:
        resetDecoder();
        break;

      case START:
        if (long_pulse) { //long pulse in preamble
          ++flip;         //expect about 32 transitions
        } else if (flip > 20) { //short pulse, start bit
          flip = 0;
          state = WAIT;
          return 0;
        } else { //short pulse within preamble, not valid
          resetDecoder();
          return 0;
        }
        break;

      case NEXT_BIT:
        if (long_pulse) {
          flip = flip ^ 1; //flip the bit
          putBit(flip);
        } else { // short pulse
          state = WAIT;  //wait for one more short pulse
        }
        break;

      case WAIT: //wsiting for second short pulse
        if (long_pulse == 0) {
          // Second short pulse
          putBit(flip);  //don't flip, put bit and set state to NEXT_BIT
        } else { // long not expected here
          resetDecoder();
          return 0;
        }
    }  //end switch state

    if (total_edges > 159) { //80 bits
      done();
      return 1; //enough edges for these two sensors
    }
    return 0; //valid pulse, more to come?
  }  //end if (200 <= width && width < 1250)

  if (width >= 1600  && index >= 8) {  //end of record gap
    done();
    return 1;
  }

  //not a valid pulse ( 200 < pulse < 1250 ) nor end of message gap
  resetDecoder();
  return 0;
}

//calculate expected checksum digit for THN132N, found in data[6] top nibble
//off by 1 for THRN122N? (should include channel, fixed at 1?)

unsigned char calcChecksum(void)
{
  unsigned char checksum;
  unsigned char i;

  // sum nibbles of rolling code and temperature+sign
  checksum = 0;

  for (i = 3; i < 6; i++)
  {
    checksum += (data[i] & 0xF) + ((data[i] >> 4)  & 0xF);
  }
  checksum += data[6] & 0x0F; //sign digit (8 or 0)
  return (checksum & 0x0F);
}

// use checksum to determine if this is likely to be a valid message
// "type" is sensor type. 2 = THN132N; 1 = THRN122N
// could also check sensor ID = EA4C (bytes 0 & 1 of message) for both

unsigned char valid_message(unsigned char type) {
  unsigned char x = calcChecksum();
  unsigned char y = (data[6] & 0xF0) >> 4; //top nibble of message
  switch (type) {
    case 2: if (x == y) {
        return 1;
        break;
      }
    case 1: if ( ((y + 1) & 0x0F) == x) return 1; //hack for THRN122N
    default: return 0;
  }
}

// get the temperature from data record, in degrees C

float temperature(void)
{
  int sign = (data[6] & 0x8) ? -1 : 1; //OK, SJR 12/2018
  float temp = (float) (((data[5] & 0xF0) >> 4) * 10 + (data[5] & 0xF)) + (float)(((data[4] & 0xF0) >> 4) / 10.0);
  float result = sign * temp;
  //dump temp bytes in HEX in order
  //if(data[4]<16) Serial.print('0');Serial.print(data[4],HEX);
  //if(data[5]<16) Serial.print('0');Serial.println(data[5],HEX);
  return result;  //degrees C
}

// get battery low flag
// return 1 (true) if low

byte battery(void)
{
  char BatteryLevel = (data[4] & 0x04) ? 1 : 0;
  return BatteryLevel;
}

//Return device rolling code from message

byte rc(void)
{
  return (data[3]);
}

//return channel # from message

byte channel(void)
{
  byte channel;
  switch (data[2])
  {
    case 0x10:
      channel = 1;
      break;
    case 0x20:
      channel = 2;
      break;
    case 0x40:
      channel = 3;
      break;
    default:
      channel = 0;
  }
  return channel;
}

// return trend bits, at least for THN132N

unsigned char trend(void) {
  return data[4] & 0x03; //2=falling, 1=rising, 0=constant
}

// hex dump of record received

void dump_record(void) {
  for (byte i = 0; i < index; ++i) {
    Serial.print(data[i] >> 4, HEX);
    Serial.print(data[i] & 0x0F, HEX);
  }
  Serial.println();
  //  debug checksum digit
  //  Serial.print(" ");
  //  Serial.print((data[6] & 0xF0) >> 4, HEX);
  //  Serial.print(" ");
  //  Serial.println(calcChecksum(), HEX);
}
#endif
