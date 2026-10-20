// Arduino Uno decoder for LaCrosse TX141_BV4 temperature sensor
// packet analysis and code written with much help from Claude, Sonnet 5.5 
// S. J. Remington 10/19/2026
// Pulse timing differs from that determined for previous versions of TX141 sensors, e.g. TX141_BV2
// See https://github.com/merbanan/rtl_433/blob/master/src/devices/lacrosse_tx141x.c
//
// Assumes data input = 433 MHz OOK RF ON = HIGH (5v), YXR15 receiver used in development.
/*
 Timing

The sensor’s clock is about 17% slower than the Bv2 timing. Everything is a multiple of one unit, T ≈ 244 µs:

Element Measured  Units
Sync pulse  H ≈ 976, L ≈ 976  4T high, 4T low
Data bit “0”  H ≈ 240, L ≈ 490  1T high, 2T low
Data bit “1”  H ≈ 490, L ≈ 245  2T high, 1T low

Each data bit lasts 3T ≈ 735 µs, and every sync pair is 8T ≈ 1950 µs.

TX141_BV4 sends four packets with each transmission

TX141_BV4 packet is four bytes plus 1 trailing 0 bit:

- id:    8 bit random integer, presumably generated at each power up
- flags: 4 bit for battery low indicator, test button press, and channel
- temp: 12 bit unsigned temperature in degrees Celsius, scaled by 10, offset 500, range -40 C to 60 C
- chk:   8 bit checksum as byte sum (see below)
- 0 bit

Flags nibble 0x9 (1001):
The 0x8 bit is set on all packets from sensors with fresh batteries and no low-battery warning. Treat it as “battery OK” until you see it clear on a weak battery.
The 0x4 bit appeared only on the button-press packet, so it’s probably “test/forced transmission”.
The 0x1 bit is set in all packets, sensor ID=1 presumed (Display supports optional sensor IDs 2 & 3)

Checksum:
A byte sum in which the nibbles are added separately, with no carry from the low nibble into the high nibble:

High nibble of byte 3 = (sum of the high nibbles of bytes 0, 1 and 2 + 6) mod 16
Low nibble of byte 3 = (sum of the low nibbles of bytes 0, 1 and 2 + 8) mod 16
 */

const uint8_t RX_PIN = 2;  //external interrupt pin
const uint8_t BUF_SIZE = 64;
volatile uint16_t edgeBuf[BUF_SIZE];
volatile uint8_t head = 0, tail = 0;
volatile uint32_t lastEdge = 0;

void onEdge() {
  uint32_t now = micros();
  uint32_t d = now - lastEdge; lastEdge = now;
  if (d > 0x7FFF) d = 0x7FFF;
  bool nowHigh = digitalRead(RX_PIN);
  uint16_t v = (uint16_t)d | (nowHigh ? 0 : 0x8000);   // bit15 = ended pulse was HIGH
  uint8_t next = (head + 1) & (BUF_SIZE - 1);
  if (next != tail) { edgeBuf[head] = v; head = next; }
}

// Timing windows (us), from measured: sync 976, short ~240, long ~490
const uint16_t SYNC_MIN = 750, SYNC_MAX = 1500;
const uint16_t BIT_MIN  = 130, BIT_SPLIT = 365, BIT_MAX = 700;
const uint16_t GAP_MAX  = 1300;     // a low longer than this ends the packet
const uint8_t  MIN_SYNC = 2;        // first sync pulse is sometimes lost
const uint8_t  PKT_BITS = 32;       // ID(8) + flags/temp(16) + checksum(8)

enum State { IDLE, SYNC, DATA };
State state = IDLE;
uint8_t syncCount = 0, nbits = 0;
uint8_t pkt[4];
uint8_t prevPkt[4] = {0, 0, 0, 0};
bool havePrev = false;

void resetAll() { state = IDLE; syncCount = 0; nbits = 0; memset(pkt, 0, 4); }

void hex2(uint8_t b) { if (b < 0x10) Serial.print('0'); Serial.print(b, HEX); }

// calculate packet checksum and compare with packet value
bool checksumOk() { 
  uint8_t hi = ((pkt[0] >> 4) + (pkt[1] >> 4) + (pkt[2] >> 4) + 6) & 0x0F;
  uint8_t lo = ((pkt[0] & 15) + (pkt[1] & 15) + (pkt[2] & 15) + 8) & 0x0F;
  return ((hi << 4) | lo) == pkt[3];
}

uint32_t lastReport = 0;

void report() {
  if (!checksumOk()) return;                       // drop corrupted packets

  bool same = havePrev && memcmp(pkt, prevPkt, 4) == 0;
  memcpy(prevPkt, pkt, 4); havePrev = true;
  uint32_t now = millis();
  if (same && (now - lastReport) < 10000UL) return;  // suppress repeats within 10 s
  lastReport = now;

  uint16_t raw = ((uint16_t)(pkt[1] & 0x0F) << 8) | pkt[2];
  float c = (raw - 500) / 10.0;

  Serial.print(F("id=0x")); hex2(pkt[0]);
  Serial.print(F("  flags=0x")); Serial.print(pkt[1] >> 4, HEX);
  Serial.print(F("  T=")); Serial.print(c, 1); Serial.print(F(" C  "));
  Serial.print(c * 9.0 / 5.0 + 32.0, 1); Serial.println(F(" F"));
}
void addBit(bool b) {
  if (b) pkt[nbits >> 3] |= (1 << (7 - (nbits & 7)));
  if (++nbits >= PKT_BITS) { report(); resetAll(); }
}

void handle(bool high, uint16_t d) {
  if (high) {
    if (d >= SYNC_MIN && d <= SYNC_MAX) {            // sync pulse
      if (state == DATA) { resetAll(); }             // new packet began: restart
      if (syncCount < 255) syncCount++;
      state = SYNC;
    } else if (state != IDLE && syncCount >= MIN_SYNC && d >= BIT_MIN && d <= BIT_MAX) {
      state = DATA;
      addBit(d >= BIT_SPLIT);                        // long high = 1, short high = 0
    } else {
      resetAll();
    }
  } else if (d > GAP_MAX) {
    resetAll();
  }
}

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
