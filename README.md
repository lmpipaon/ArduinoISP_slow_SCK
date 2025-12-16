ArduinoISP slow SCK
===================


This is a modification of the Arduino ISP sketch that allows a lower SCK frequency to program microcontrollers with slow clocks, for example when using the watchdog clock divided by 8.
When programming mode starts, ArduinoISP uses a low SCK frequency by setting the system clock prescaler.
When programming mode ends, the system clock prescaler is set back to 1.
