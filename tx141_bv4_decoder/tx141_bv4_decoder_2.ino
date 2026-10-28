/*
  LaCrosse TX141-BV4 temperature sensor decoder
  Arduino Uno + YXR15 433 MHz OOK receiver, DATA pin -> D2 (INT0), data output LOW = carrier ON
  For receivers with data output = HIGH for carrier ON, set RX_ACTIVE_LOW to false (below)
  OK 10/28/2026
  
  S.J. Remington with the help of Claude (Sonnet 5.5) 10/2026

  Protocol (as measured):
    - OOK, pulse-width coded, time unit T ~ 244 us
    - Sync pulse: ~976 us high + ~976 us low (4T/4T), 3-4 per packet
    - Bit "1": ~240 us high + ~490 us low   Bit "0": ~490 us high + ~245 us low 
    - Payload: 32 bits, MSB first, [acket has 33rd trailing bit=0 (ignored).
        byte 0 : sensor ID (changes when the battery is replaced)
        byte 1 : flags nibble (high) + temperature bits 11..8 (low)
        byte 2 : temperature bits 7..0
        byte 3 : checksum
    - Temperature = (raw12 - 500) / 10  deg C
    - Checksum = swapNibbles( swapNibbles(b0) + swapNibbles(b1)
                              + swapNibbles(b2) + 0x66 )   (mod 256)
    - Flags: 0x8 bit set on all packets so far (assumed battery OK),
             0x4 bit set on button-forced transmissions
    - Each reading is sent as a burst of ~4 packets
*/

const uint8_t RX_PIN = 2;                 // external interrupt pin
// true if the receiver's DATA pin goes LOW while a carrier is present
const bool RX_ACTIVE_LOW = true;

// ---------------------------------------------------------------------------
// Edge capture (interrupt)
// ---------------------------------------------------------------------------
const uint8_t BUF_SIZE = 64;              // must be a power of 2
volatile uint16_t edgeBuf[BUF_SIZE];      // bit 15 = ended pulse was HIGH
volatile uint8_t head = 0, tail = 0;
volatile uint32_t lastEdge = 0;

long bad_chksums = 0;

void onEdge() {
  uint32_t now = micros();
  uint32_t d = now - lastEdge;
  lastEdge = now;
  if (d > 0x7FFF) d = 0x7FFF;
  bool carrierNow = (digitalRead(RX_PIN) == HIGH) != RX_ACTIVE_LOW;
  uint16_t v = (uint16_t)d | (carrierNow ? 0 : 0x8000);   // bit 15 = ended interval had carrier ON
  uint8_t next = (head + 1) & (BUF_SIZE - 1);
  if (next != tail) { edgeBuf[head] = v; head = next; }
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------
// Timing windows (us)
const uint16_t SYNC_MIN = 750, SYNC_MAX = 1500;     // sync high pulse
const uint16_t BIT_MIN = 130, BIT_SPLIT = 365, BIT_MAX = 700;  // data high pulse
const uint16_t GAP_MAX = 1300;                      // longer low = end of packet
const uint8_t  MIN_SYNC = 2;                        // first sync pulse is sometimes lost
const uint8_t  PKT_BITS = 32;
const uint32_t REPEAT_HOLDOFF_MS = 10000UL;         // suppress identical repeats

enum State { IDLE, SYNC, DATA };
State state = IDLE;
uint8_t syncCount = 0, nbits = 0;
uint8_t pkt[4];
uint8_t prevPkt[4];
bool havePrev = false;
uint32_t lastReport = 0;

void resetAll() {
  state = IDLE;
  syncCount = 0;
  nbits = 0;
  memset(pkt, 0, sizeof(pkt));
}

static inline uint8_t swapNibbles(uint8_t b) {
  return (uint8_t)((b << 4) | (b >> 4));
}

bool checksumOk() {
  uint8_t s = swapNibbles(pkt[0]) + swapNibbles(pkt[1]) + swapNibbles(pkt[2]) + 0x66;
  return swapNibbles(s) == pkt[3];
}

void hex2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void report() {
  if (!checksumOk()) { bad_chksums++; return; }                       // drop corrupted packets

  bool same = havePrev && memcmp(pkt, prevPkt, sizeof(pkt)) == 0;
  memcpy(prevPkt, pkt, sizeof(pkt));
  havePrev = true;

  uint32_t now = millis();
  if (same && (now - lastReport) < REPEAT_HOLDOFF_MS) return;
  lastReport = now;

  int16_t raw = ((int16_t)(pkt[1] & 0x0F) << 8) | pkt[2];   // signed: avoids wrap below 0 C
  float c = (raw - 500) / 10.0;

  Serial.print(F("id=0x")); hex2(pkt[0]);
  Serial.print(F("  flags=0x")); Serial.print(pkt[1] >> 4, HEX);
  Serial.print(F("  T= ")); 
  // Serial.print(c, 1);
  // Serial.print(F(" C  ")); 
  Serial.print(c * 9.0 / 5.0 + 32.0, 1);
  Serial.print(F(" F #"));
  Serial.println(bad_chksums);
}

void addBit(bool b) {
  if (b) pkt[nbits >> 3] |= (1 << (7 - (nbits & 7)));
  if (++nbits >= PKT_BITS) {
    report();
    resetAll();
  }
}

void handle(bool carrierOn, uint16_t d) {
  if (carrierOn) {
    if (d >= SYNC_MIN && d <= SYNC_MAX) {            // sync: carrier on ~976 us
      if (state == DATA) resetAll();                // a new packet began mid-data
      if (syncCount < 255) syncCount++;
      state = SYNC;
    } else if (state != IDLE && syncCount >= MIN_SYNC && d >= BIT_MIN && d <= BIT_MAX) {
      state = DATA;
      addBit(d <= BIT_SPLIT);   // carrier-on pulse: short (~240 us) = 1, long (~490 us) = 0
    } else {
      resetAll();
    }
  } else if (d > GAP_MAX) {     // carrier off longer than GAP_MAX: idle / end of packet
    resetAll();
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);
  lastEdge = micros();
  attachInterrupt(digitalPinToInterrupt(RX_PIN), onEdge, CHANGE);
  Serial.println(F("TX141 decoder ready"));
}

void loop() {
  while (tail != head) {
    uint16_t v = edgeBuf[tail];
    tail = (tail + 1) & (BUF_SIZE - 1);
    handle(v & 0x8000, v & 0x7FFF);
  }
}
