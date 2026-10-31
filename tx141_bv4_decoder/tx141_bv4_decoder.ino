/*
  LaCrosse TX141-BV4 temperature sensor decoder
  Arduino Uno + YXR15 433 MHz OOK receiver, DATA pin -> D2 (INT0)
  This receiver has data output LOW = carrier on

  S.J. Remington and Claude (Sonnet 5.5) 10/20/2026

  Protocol (as measured):
    - OOK, pulse-width coded, time unit T ~ 244 us
    - Sync pulse: ~976 us high + ~976 us low (4T/4T), 4 per packet
    - Bit "1": ~245 us high (carrier ON) + ~490 us low (carrier OFF)
    - Bit "0": ~490 us high + ~245 us low
    - Payload: 32 bits, MSB first, packet has 33rd trailing bit=0 (ignored).
        byte 0 : sensor ID (changes when the battery is replaced)
        byte 1 : flags nibble (high) + temperature bits 11..8 (low)
        byte 2 : temperature bits 7..0
        byte 3 : checksum
    - Temperature = (raw12 - 500) / 10  deg C
    - Checksum = swapNibbles( swapNibbles(b0) + swapNibbles(b1)
                              + swapNibbles(b2) + 0x66 )   (mod 256)
    - Flags: 0x8 bit set on all packets so far (assumed battery OK),
             0x4 bit set on button-forced transmissions
             0x1 bit set on all transmissions, assumed this is sensor #1
    - Each reading is sent as two bursts of 4 identical packets, ~20 ms gap between bursts
    - Reading sent about every 30 seconds
*/

const uint8_t RX_PIN = 2;                 // external interrupt pin

// ---------------------------------------------------------------------------
// Edge capture (interrupt)
// ---------------------------------------------------------------------------
const int BUF_SIZE = 256;              // must be a power of 2
volatile uint16_t edgeBuf[BUF_SIZE];      // bit 15 = ended pulse was HIGH
volatile uint16_t head = 0, tail = 0;
volatile uint32_t lastEdge = 0;

long bad_chksums = 0;

void onEdge() {
  uint32_t now = micros();
  uint32_t d = now - lastEdge;
  lastEdge = now;
  if (d > 0x7FFF) d = 0x7FFF;
  bool carrierOn = !digitalRead(RX_PIN);  //for inverting receiver, carrierOn = 1 for LOW
  uint16_t v = (uint16_t)d | (carrierOn ? 0 : 0x8000); //store bit state of PREVIOUS pulse
  uint16_t next = (head + 1) & (BUF_SIZE - 1);
  if (next != tail) {                     // drop edge if buffer is full
    edgeBuf[head] = v;
    head = next;
  }
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------
// Timing windows (us)
const uint16_t SYNC_MIN = 800, SYNC_MAX = 1200;     // sync high pulse
const uint16_t BIT_MIN = 130, BIT_SPLIT = 365, BIT_MAX = 700;  // data high pulse
const uint16_t GAP_MAX = 1300;                      // longer low = end of packet
const uint8_t  MIN_SYNC = 3;                        // 3 sync pairs
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
uint8_t chksum = 0;
bool checksumOk() {
  uint8_t s = swapNibbles(pkt[0]) + swapNibbles(pkt[1]) + swapNibbles(pkt[2]) + 0x66;
  chksum = swapNibbles(s);
  return chksum == pkt[3];
}

void hex2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void report() {
  if (!checksumOk()) { // print and drop corrupted packets
    bad_chksums++;
    for (int i = 0; i < 4; i++) {
      hex2(pkt[i]);
      Serial.print(' ');
    }
    hex2(chksum);
    Serial.println();
    return;
  }

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
// handle bit interpretation. high = 1 => carrier ON state

void handle(bool high, uint16_t d) {
  if (high) { 
    if (d >= SYNC_MIN && d <= SYNC_MAX) {           // sync pulse
      if (state == DATA) resetAll();                // a new packet began mid-data
      if (syncCount < 255) syncCount++;
      state = SYNC;
    } else if (state != IDLE && syncCount >= MIN_SYNC && d >= BIT_MIN && d <= BIT_MAX) {
      state = DATA;
      addBit(d < BIT_SPLIT);                       // short carrier ON pulse = 1 for TX141-BV4
      } else {
      resetAll();
    }
  } else if (d > GAP_MAX) {
    resetAll();
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(230400);
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
