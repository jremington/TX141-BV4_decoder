// receiver YXR15 inverts (active LOW = carrier ON)
/* 
 *  typical output from TX141-BV4 sensor (clean dump)
 --- packet ---
H976 L976 H980 L980 H972 L976 H984 L964
H500 L240 H492 L248 H236 H488 L248 H484
L248 H244 L488 H240 L500 H236 L488 H244
L492 H240 L488 H240 L492 H244 L484 H492
L244 H492 L240 H488 L248 H240 L488 H488
L248 H244 L484 H492 L244 H240 L548 H980
L968 H984 L968 H984 L972 H976 L980 H488
L248 H488 L244 H244 L492 H240 L488 H484
L248 H244 L496 H484 L244 H480 L252 H484
L248 H236 L496 H240 L488 H488 L248 H240
L488 H248 L488 H484 L252 H240 L488 H488
L248 H484 L244 H240 L496 H240 L484 H252

 */
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
  uint16_t v = (uint16_t)d | (nowHigh ? 0 : 0x8000);   // bit15 = ended pulse was HIGH
  uint8_t next = (head + 1) & (BUF_SIZE - 1);
  if (next != tail) { edgeBuf[head] = v; head = next; }
}

// Sync acceptance windows (us), widened to match your Audacity measurements
const uint16_t SYNC_H_MIN = 700,  SYNC_H_MAX = 1300;
const uint16_t SYNC_L_MIN = 700,  SYNC_L_MAX = 1300;
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
