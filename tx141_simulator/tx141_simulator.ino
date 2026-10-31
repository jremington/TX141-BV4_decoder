/*
  LaCrosse TX141-BV4 sensor simulator
  Arduino Uno + 433.92 MHz OOK transmitter module, DATA input -> D2 (HIGH = carrier on)
  S. J. Remington 10/31/2026
  
  Sends the TX141-BV4 packet format decoded from a real sensor (see tx141_decoder.ino):
    - Sync: 4 pulses of ~976 us high + ~976 us low
    - Bit "1": ~244 us high + ~488 us low,  Bit "0": ~488 us high + ~244 us low
    - 32-bit payload, MSB first:  ID | flags nibble + 12-bit temp (high) | temp (low) | checksum
    - One trailing "0" bit
    - Temperature field = tenths of a degree C + 500
    - Checksum = swapNibbles( swapNibbles(b0) + swapNibbles(b1) + swapNibbles(b2) + 0x66 )
    - Each reading is sent as a burst of identical back-to-back packets

  Note: this packet has no humidity field. The BV4 is a temperature-only model, and I have
  not seen the layout of a temperature + humidity variant, so none is simulated here.
*/

const uint8_t TX_PIN = 2;
const bool INVERT_TX = false;   // if true, simulate receiver output LOW while carrier is on

// ---- Simulated sensor settings --------------------------------------------
const uint8_t SENSOR_ID    = 0xA5;   // real sensors pick a new random ID at battery insertion
const uint8_t CHANNEL_BITS = 0x01;   // flags bits 1:0. Channel 1 = 0x01 (observed); 0x02 for
// channel 2 is an assumption, not yet seen on a real unit
const bool    BATTERY_OK   = true;   // flags bit 0x8 (set on every real packet seen so far)
const bool    FORCED_TX    = false;  // flags bit 0x4 (set on button-forced transmissions)

const uint8_t  REPEATS     = 2;      // identical packets in one burst per reading (real sensor: 4)
const uint32_t INTERVAL_MS = 10000UL;// pause between readings (real sensor: ~30 s)

// Test temperatures in tenths of a degree C, chosen to cross the values that exposed
// carry behaviour in the checksum: sub-zero, 0, ~9.2, ~26.8, ~28.4
const int16_t TEMPS_TENTHS[] = {
  -100, -53, -4, 0, 9, 42, 91, 93, 100, 229, 267, 268, 284, 350
};
const uint8_t N_TEMPS = sizeof(TEMPS_TENTHS) / sizeof(TEMPS_TENTHS[0]);

// ---- Timing (microseconds) --------------------------------------------------
const uint16_t T_US        = 244;
const uint16_t SYNC_US     = 4 * T_US;   // 976
const uint8_t  SYNC_PULSES = 4;

// ---------------------------------------------------------------------------
static inline uint8_t swapNibbles(uint8_t b) {
  return (uint8_t)((b << 4) | (b >> 4));
}

uint8_t checksum(uint8_t b0, uint8_t b1, uint8_t b2) {
  uint8_t s = swapNibbles(b0) + swapNibbles(b1) + swapNibbles(b2) + 0x66;
  return swapNibbles(s);
}

void buildPacket(int16_t tenths, uint8_t p[4]) {
  int16_t raw = tenths + 500;                      // 12-bit field, 0..4095
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  uint8_t flags = (BATTERY_OK ? 0x8 : 0) | (FORCED_TX ? 0x4 : 0) | (CHANNEL_BITS & 0x3);
  p[0] = SENSOR_ID;
  p[1] = (uint8_t)((flags << 4) | ((raw >> 8) & 0x0F));
  p[2] = (uint8_t)(raw & 0xFF);
  p[3] = checksum(p[0], p[1], p[2]);
}


void pulse(uint16_t highUs, uint16_t lowUs) {
  digitalWrite(TX_PIN, INVERT_TX ? LOW : HIGH);
  delayMicroseconds(highUs);
  digitalWrite(TX_PIN, INVERT_TX ? HIGH : LOW);
  delayMicroseconds(lowUs);
}

void sendBit(bool one) {
  if (one) pulse(T_US, 2*T_US);
  else     pulse(2*T_US, T_US);
}

void sendPacket(const uint8_t p[4]) {
  // +/- 100 allows to distinguish SYNC H/L for receiver calibration, passes length test in decoder
  for (uint8_t i = 0; i < SYNC_PULSES; i++) pulse(SYNC_US + 100, SYNC_US - 100); 
  for (uint8_t i = 0; i < 32; i++) sendBit(p[i >> 3] & (0x80 >> (i & 7)));
  pulse(2*T_US, T_US);  // trailing "0" bit
}

void hex2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

// Check the checksum routine against packets captured from the real sensor
void selfTest() {
  static const uint8_t vec[][4] = {
    {0xCA, 0x92, 0xD9, 0x8D},   // 22.9 C
    {0xCA, 0xD2, 0xD9, 0xCD},   // 22.9 C, button-forced
    {0xCA, 0x91, 0xF0, 0xA3},   // -0.4 C
    {0xCA, 0x92, 0x4F, 0xF2},   // 9.1 C
    {0xCA, 0x93, 0x10, 0xC4},   // 28.4 C
  };
  bool ok = true;
  for (uint8_t i = 0; i < sizeof(vec) / sizeof(vec[0]); i++)
    if (checksum(vec[i][0], vec[i][1], vec[i][2]) != vec[i][3]) ok = false;
  Serial.println(ok ? F("Checksum self-test: PASS") : F("Checksum self-test: FAIL"));
}

void printTenths(int16_t t) {
  if (t < 0) {
    Serial.print('-');
    t = -t;
  }
  Serial.print(t / 10); Serial.print('.'); Serial.print(t % 10);
}

void setup() {
  Serial.begin(115200);
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);                       // idle = carrier off
  Serial.println(F("TX141 simulator ready"));
  selfTest();
}

void loop() {
  static uint8_t idx = 0;
  uint8_t p[4];
  buildPacket(TEMPS_TENTHS[idx], p);

  // Print first and wait for the UART to finish: serial interrupts would disturb pulse timing
  Serial.print(F("Sending id=0x")); hex2(p[0]);
  Serial.print(F(" flags=0x")); Serial.print(p[1] >> 4, HEX);
  Serial.print(F(" T=")); printTenths(TEMPS_TENTHS[idx]);
  Serial.print(F(" C  bytes: "));
  for (uint8_t i = 0; i < 4; i++) {
    hex2(p[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.flush();

  for (uint8_t r = 0; r < REPEATS; r++) sendPacket(p);
  digitalWrite(TX_PIN, LOW);

  idx = (idx + 1) % N_TEMPS;
  delay(INTERVAL_MS);
}
