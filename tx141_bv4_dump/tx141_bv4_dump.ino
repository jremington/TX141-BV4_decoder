// packet dump routine generalized for non-inverting and inverting receivers
// for non-inverting receiver, select "false" for RX_ACTIVE_LOW

const uint8_t RX_PIN = 2;
// true if the receiver's DATA pin goes LOW while a carrier is present
const bool RX_ACTIVE_LOW = true;

const uint8_t BUF_SIZE = 128;
volatile uint16_t edgeBuf[BUF_SIZE];
volatile uint8_t head = 0, tail = 0;
volatile uint32_t lastEdge = 0;

// interrupt routine

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

// Sync acceptance windows (us)
const uint16_t SYNC_MIN = 700,  SYNC_MAX = 1300;
const uint8_t  SYNC_PAIRS = 3;     // consecutive sync pairs required to trigger
const uint8_t  CAP_LEN    = 96;    // total edges to record (incl. sync)

uint16_t cap[CAP_LEN];
uint8_t n = 0;

void dumpCapture() {
  Serial.println(F("--- packet ---"));
  for (uint8_t i = 0; i < CAP_LEN; i++) {
    Serial.print((cap[i] & 0x8000) ? 'H' : 'L');  // H means carrier ON
    Serial.print(cap[i] & 0x7FFF);
    Serial.print(i % 8 == 7 ? "\r\n" : " ");
    if((cap[i]&0x7FFF) == 0x7FFF) break;
  }
  Serial.println();
}

void handleEdge(bool carrierOn, uint16_t d) {
  if (n < SYNC_PAIRS * 2) {
    // Still looking for a run of sync pairs: carrier ON at even index, OFF at odd
    bool wantHigh = (n % 2 == 0);
    bool ok = false;
    if (wantHigh && carrierOn && d >= SYNC_MIN && d <= SYNC_MAX) ok = true;
    if (!wantHigh && !carrierOn && d >= SYNC_MIN && d <= SYNC_MAX) ok = true;

    if (ok) {
      cap[n++] = d | (carrierOn ? 0x8000 : 0);
    } else {
      // Pattern broken: restart, but this edge may itself begin a new sync run
      n = 0;
      if (carrierOn && d >= SYNC_MIN && d <= SYNC_MAX) cap[n++] = d | 0x8000;
    }
  } else {
    // Sync confirmed: record everything until the buffer is full
    cap[n++] = d | (carrierOn ? 0x8000 : 0);
    if (n >= CAP_LEN) {
      dumpCapture();
      n = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);
  lastEdge = micros();
  attachInterrupt(digitalPinToInterrupt(RX_PIN), onEdge, CHANGE);
  Serial.println(F("Waiting for sync pattern..."));
}

void loop() {
  while (tail != head) {
    uint16_t v = edgeBuf[tail];
    tail = (tail + 1) & (BUF_SIZE - 1);
    handleEdge(v & 0x8000, v & 0x7FFF);
  }
}
