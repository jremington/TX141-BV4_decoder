Arduino Uno C/C++ decoder for LaCrosse TX141-BV4 temperature sensor.

Packet timing and content analysis performed and code written with much help from Claude, Sonnet 5.5

Receiver: 433 MHz OOK YXR15, data output = HIGH for TX carrier ON, connected to external interrupt pin 2, 5V power and GND.

S. J. Remington 10/19/2026

Note: Pulse timing, packet size and checksum calculation differs from that determined for previous versions of TX141 sensors, e.g. TX141_BV2
See https://github.com/merbanan/rtl_433/blob/master/src/devices/lacrosse_tx141x.c
