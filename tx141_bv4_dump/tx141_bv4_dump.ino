// NOTE Serial baud rate 230400!
// capture and dump 433 MHz temp sensor transmissions.
// receiver YXR15 inverts (confirmed active LOW = carrier ON) using simulator and FS1000A transmitter
// new sync routine
// here, define HIGH or H as carrier ON

const uint8_t RX_PIN = 2;
const int BUF_SIZE = 256;
volatile uint16_t edgeBuf[BUF_SIZE];
volatile unsigned int head = 0, tail = 0;
volatile uint32_t lastEdge = 0;

void onEdge() {
  uint32_t now = micros();
  uint32_t d = now - lastEdge; lastEdge = now;
  if (d > 0x7FFF) d = 0x7FFF;
  
  //get pin state after transition
  bool carrierOn = !digitalRead(RX_PIN);  //for inverting receiver (data LOW = carrier ON)
  
  uint16_t v = (uint16_t)d | (carrierOn ? 0 : 0x8000);   //, invert, bit15 = last pulse: 1 = carrier ON
  unsigned int next = (head + 1) & (BUF_SIZE - 1);
  if (next != tail) { edgeBuf[head] = v; head = next; }
}

// Sync acceptance windows (us)
const uint16_t SYNC_MIN = 800,  SYNC_MAX = 1200;
const uint8_t  SYNC_PAIRS = 3;     // consecutive sync pairs required to trigger
const int  CAP_LEN    = 400;    // total edges to record (incl. sync)

uint16_t cap[CAP_LEN];
int n = 0;

void dumpCapture() {
  Serial.println(F("--- packet ---"));
  for (int i = 0; i < CAP_LEN; i++) {
    Serial.print((cap[i] & 0x8000) ? 'H' : 'L');
    Serial.print(cap[i] & 0x7FFF);
    Serial.print(i % 8 == 7 ? "\r\n" : " ");
    if ((cap[i]&0x7fff) > 5000) break;
  }
  Serial.println();
}

void handleEdge(bool high, uint16_t d) {
  if (n < SYNC_PAIRS * 2) {
    // Still looking for a run of sync pairs
    bool ok = false;
    if (high && d >= SYNC_MIN && d <= SYNC_MAX) ok = true;
    if (!high && d >= SYNC_MIN && d <= SYNC_MAX) ok = true;

    if (ok) {
      cap[n++] = d | (high ? 0x8000 : 0);
    } else {
      // Pattern broken: restart, but this edge may itself begin a new sync run
      n = 0;
      if (high && d >= SYNC_MIN && d <= SYNC_MAX) cap[n++] = d | 0x8000;
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
  Serial.begin(230400);
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
