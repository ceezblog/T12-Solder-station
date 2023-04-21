# T12-Solder-station

Just another Solder station using ATMega328 (or variant)

## 1. Main board
- Support only T12 tip that has 4ohm heater
- Only work with ATMega168, ATMega328 or newer ATMega328PB because code uses very specific timer as well as interrupt pins for the encoder.
- Use IRF4905 (P-channel) to turn on/off heat element
- Use LMV321 for amplifier Thermocouple voltage
- Using buck converter and linear regulator for high efficiency
- Use homemade opto switch, just a pair of IR LED and IR Reciever Diode (or phototransistor)
- Encoder to control the temperature
<img src='PCB/Diagram.png' width='40%' alt='schematic'/>
Pdf version here <a href='SolderStation_r0.4.pdf'>SolderStation_r0.4.pdf</a>


## 2. Switching PSU
- The 3D printed enclosure only support this power supply WX-DC2412_24V_4A, search aliexpress for "WX-DC2412 24V 4A"
- There are a few variants of this power supply that has the same mounting holes
<img src='PSU/WX-DC2412_24V_4A.jpg' width=30% alt='24V-4A switching power supply'/>


## 4. Display
- Only support LCD0802 - 8 char 2 line Character LCD
- The main board was designed to piggyback on the LCD
- It is possible to use SSD1306 display via I2C but I haven't written code for this
 <img src='LCD0802/lcd0802.jpg' width=30% alt='LCD0802'/>


## 5. Enclosure
- All 3D printed, except for the rubber feet to stop this from sliding when pressing power button
- Include stand for the handle
- Include opto switch housing, just need a pair of IR LED and IR receiver diode.
 <img src='Enclosure/T12 box.png' width=600px alt='3D printed Enclosure'/>


## 6. Firmware for the main micro
- Low level code to config timer, pwm, interrupts...
- Only three libraries are used: **avr/interrupt.h, EEPROM.h and LiquidCrystal.h**

