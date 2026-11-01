Arduino C/C++ decoder for LaCrosse TX141-BV4 temperature sensor.

Packet timing and content analysis were performed and code was written with help from Claude.ai, Sonnet 5.5

Receiver used: 433 MHz OOK YXR15, data output = LOW for TX carrier ON, connected to external interrupt pin 2, 5V power and GND. Tested on Arduino Uno.

S. J. Remington 10/19/2026

Note: Pulse timing, packet size and checksum calculation (not a CRC) differ substantially from that determined for previous versions of TX141 sensors, e.g. TX141-BV2
See https://github.com/merbanan/rtl_433/blob/master/src/devices/lacrosse_tx141x.c

Screenshot of Pulseview capture of a TX141-BV4 packet. Receiver used here is an RXB6 superhet (inverting, LOW = carrier ON) connected directly to logic analyzer.

<img width="1177" height="372" alt="TX141-BV4_capture" src="https://github.com/user-attachments/assets/5b4aed3d-3a88-4d1f-8c1e-ce467a07a791" />



