# BLE enabled Rotary Encoder with Pushbutton and 2 LEDs

Date: 5. Mai 2015
Author: Sebastian Herp

## Usage on Linux/Raspberry Pi (Bluez 5.30):

1) `hciconfig` => if it shows the interface to be down execute "sudo hciconfig hci0 up"
2) `sudo hcitool lescan` to scan for BLE devices
3) `gatttool -t random -b F0:0E:BD:D9:5B:3D --interactive` to start gatttool
4a) `connect` => command line should change to blue and you should be connected to the Blend Micro with the address F0:0E:BD:D9:5B:3D
4b) `char-write-req 0x0011 0100` asks for notifications opening the return channel (status messages, encoder position and button presses are received here)
5a) `char-write-cmd 0x000e 00` asks for a status reply
5b) `char-write-cmd 0x000e 01ff00` sets the red LED to full brightness (0100ff: green LED full brightness, 018080: both LEDs half brightness, 010000: both LEDs off)
6) `disconnect` / `exit` to disconnect from device

## Replies (hex values):

Status: "00 00 06 00 00 0d 75 20 00" (encoder pos = 00 06, leds = 00 00, voltage = 0d 75, bat. percentage = 20, running on USB power = 00)
Encoder movement: "01 00 10 01 5a" (encoder pos = 00 10, speed in deg/s = 01 5a)
Button: "02 01" (button pressed) and "02 00" (button depressed)

## Power saving strategy:

Since the Blend Micro always draws a minimum of 1.7-2.0 mA ( http://redbearlab.com/blend-low-power-settings/ ) a external latching power switch is used. It is activated
by USB power, button press and the Arduino Pin D5 being HIGH.

The Blend Micro deactivates itself after a long button press or the battery running below a certain voltage for some time while not connected. It also deactivates below
~3.1 V because the gate voltage on the NPN transistor will be to low.

While running power is saved by sleeping during periods of low activity. The Arduino wont sleep when connected to USB power, a LED is on (PWM, not full brightness) and
certain other conditions to make sure debounce, etc works. The LEDs will turn off after a configurable amount of time to save power.

## Power usage:

Off: 0.0 mA
Startup: 12-16 mA
Sleeping: ~2.5 mA (BLE still on in this mode)
Encoder movement or putton press: ~6.1 mA
LEDs:
  red   = FE: 15 mA
  green = FE: 10 mA
  both  = FE: 17.4 mA
  red   = FF: 11.9 mA
  green = FF: 6.3 mA
  both  = FF: 14.7 mA

