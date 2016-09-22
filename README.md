# USB HID Data Gateway

At the moment, this project is a simple USB stack implementation for Atmel SAM D21 (D11).

Demo application is a very simple and inefficient USB HID to I2C adapter.

Demo CLI application uses I/O1 Xplained Pro board to read/write EEPROM and read temperature
from the AT30TSE785. It also tests if GPIO1 and GPIO2 pins are shorted to demo GPIO
capabilities.

I/O1 Xplained Pro has to be attached to EXT1 connector of the SAM D21 Xplained Pro board.

Future versions will improve on this and implement other interfaces.
