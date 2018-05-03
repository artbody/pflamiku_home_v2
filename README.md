# pflamiku_home_v2
plant watering system based on a Arduino Micro Pro.
There are 3 weightcells used, so we have no friction in mechanical parts.

Schematic can be found here:
https://easyeda.com/artbody/pflamiku_allinone_3er_2018_04_18

You need : one Atmega 32u4 micro pro 5V 16 MHz, one PCB from above with all parts,  3 weightcells and a well designed casing. Wire and build all together.

Next download the pflamiku_home_v2 repository, and upload the code by using Arduino IDE to your atmega 32u4 micro pro

Author: Achim Merath

* circuit Arduino micro plus
 * with 
 * HX711 Sensors and weightcell 1kg up to xxxkg
 * Motordriver 
 * LDR 
 * 
 * measure the weight of a plant and drives a waterpump if weight is under MIN until MAX is reached
 * to programm use the SW1 Led blinks and pump is running until user releases the SW1
 * Min and Max are saved in the Eeprom
 * to program the pflamiku you need at first 
 * a plant in a potery which should be watered
 * put this plant in place
 * switch on the SW1 
 * power on the device
 * wait until the LED is blinking and then wait until enough water is pumped
 * then put the SW1 in OFF position 
 * thats it your pflamiku is now programmed
 * It may be that this Steps must be repeated if the Plant is growing
 * in case the plant is growing slowly the cycle to do this steps may be half a year
 * in case of fast growing this time may be shorter
 * during the night a ldr detects darkness and so the motor is disabled
 * 
 * have fun with your pflamiku
 * 
 * 
 * 
 * ------------------ TODO ------------------
 * clean up the code
 * 
 * ------------------------------------------
 * Micro, Leonardo, other 32u4-based boards : irq is on pin 0, 1, 2, 3, 7
 * 
 * ---------------Licence
 * GNU GPL 3.0
