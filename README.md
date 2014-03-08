ArduinoISP slow SCK
===================


It is a modification of the sketch Arduino ISP allowing lower frequency of SCK to program microcontrollers with slow clocks, eg using the watchdog clock divided by 8.

When the programming mode begins ArduinoISP low frequency using the System Clock Prescaler.

When the programing mode finish the System Clock Prescaler is again set to 1
