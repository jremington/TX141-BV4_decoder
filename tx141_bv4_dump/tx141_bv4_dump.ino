// dump raw packets for timing tests, consistent with sync sequence for TX141_BV4 estimated using Audacity
// receiver on external interrupt pin 2: YXR15, DATA OUTPUT = LOW for 433 MHz carrier ON.
// S. J. Remington 10/19/2026, with help from Claude AI

const uint8_t RX_PIN = 2;
const uint8_t BUF_SIZE = 64;
volatile uint16_t edgeBuf[BUF_SIZE];
volatile uint8_t head = 0, tail = 0;
volatile uint32_t lastEdge = 0;

void onEdge() {
  uint32_t now = micros();
  uint32_t d = now - lastEdge; lastEdge = now;
  if (d > 0x7FFF) d = 0x7FFF;
  bool nowHigh = digitalRead(RX_PIN);
  uint16_t v = (uint16_t)d | (nowHigh ? 0 : 0x8000);   // inverse logic: bit15 = ended pulse was HIGH
  uint8_t next = (head + 1) & (BUF_SIZE - 1);
  if (next != tail) { edgeBuf[head] = v; head = next; }
}

// Sync acceptance windows (us), to match Audacity measurements
const uint16_t SYNC_H_MIN = 700,  SYNC_H_MAX = 1600;
const uint16_t SYNC_L_MIN = 300,  SYNC_L_MAX = 1500;
const uint8_t  SYNC_PAIRS = 3;     // consecutive sync pairs required to trigger
const uint8_t  CAP_LEN    = 96;    // total edges to record (incl. sync)

uint16_t cap[CAP_LEN];
uint8_t n = 0;

void dumpCapture() {
  Serial.println(F("--- packet ---"));
  for (uint8_t i = 0; i < CAP_LEN; i++) {
    Serial.print((cap[i] & 0x8000) ? 'H' : 'L');
    Serial.print(cap[i] & 0x7FFF);
    Serial.print(i % 8 == 7 ? "\r\n" : " ");
    if ((cap[i] & 0x7FFF) == 0x7FFF) break; //exit loop
  }
  Serial.println();
}

void handleEdge(bool high, uint16_t d) {
  if (n < SYNC_PAIRS * 2) {
    // Still looking for a run of sync pairs: high at even index, low at odd
    bool wantHigh = (n % 2 == 0);
    bool ok = false;
    if (wantHigh && high && d >= SYNC_H_MIN && d <= SYNC_H_MAX) ok = true;
    if (!wantHigh && !high && d >= SYNC_L_MIN && d <= SYNC_L_MAX) ok = true;

    if (ok) {
      cap[n++] = d | (high ? 0x8000 : 0);
    } else {
      // Pattern broken: restart, but this edge may itself begin a new sync run
      n = 0;
      if (high && d >= SYNC_H_MIN && d <= SYNC_H_MAX) cap[n++] = d | 0x8000;
    }
  } else {
    // Sync confirmed: record everything until the buffer is full
    cap[n++] = d | (high ? 0x8000 : 0);
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
