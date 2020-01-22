# ThermometerNRF
Battery powered thermometer with low power 2.4GHz connection (NRF24L01+) to a station with an LCD and a WiFi connection (ESP8266) to a web server.

## Tools
PCBs were designed with Altium designer.
Firmware for node V1 and the SensorHub/station in Atmel studio.
Firmware for node V2 with KEIL MDK.
The ESP8266 attached to the sensor hub will be programmed with arduino libraries:https://github.com/esp8266/Arduino

## Node V1
The first version of the node was supposed to be based on the ATtiny2313 uC, but I discontinued that version due to high power consumption. The PCB design is finished, but was never actually fabricated. Firmware was never finished or tested.

## Node V2
The second version of the node is based on the STM32L011 and is currently in development. The STMTestNucleo32L01 folder contains parts of the code which are being tested, as the final PCB doesn't have room for a serial link to the PC.

