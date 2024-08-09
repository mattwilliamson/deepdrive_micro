# Hardware

## MCU
Raspberry Pi Pico 
https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html

## Sensors

### HC-SR04 (HC-SR04p) Ultrasonic Rangefinder

The "p" version works with 3.3v. 

### ICM20948

9-axis IMU
https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/DMP.md
Accel/Gyro/Compass accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest. 
0 - 800 range??
Did not have time to check but my guess is that bits 0 and 1 are for one modality (say accel), then bits 4 and 5 for the next (say gyro) and 8 and 9 for the last (say compass)? Not sure about the position of the bits used (seems reasonnable to space every 4 bits). 

int16_t Accuracy;

higher seems better

Start 
474 is 00000001 11 01 10 10
432 is 00000001 10 11 00 00
422 is 00000001 10 10 01 10
420 is 00000001 10 10 01 00
419 is 00000001 10 10 00 11
407 is 00000001 10 01 01 11
389 is 00000001 10 00 01 01
373 is 00000001 01 11 01 01
367 is 00000001 01 10 11 11
360 is 00000001 01 10 10 00
365 is 00000001 01 10 11 01
363 is 00000001 01 10 10 11
347 is 00000001 01 01 10 11
346 is 00000001 01 01 10 10
345 is 00000001 01 01 10 01
339 is 00000001 01 01 00 11
333 is 00000001 01 00 11 01
329 is 00000001 01 00 10 01
318 is 00000001 00 11 11 10
317 is 00000001 00 11 11 01
315 is 00000001 00 11 10 11
314 is 00000001 00 11 10 10
308 is 00000001 00 11 01 00
307 is 00000001 00 11 00 11
306 is 00000001 00 11 00 10
304 is 00000001 00 11 00 00
298 is 00000001 00 10 10 10

bias: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/issues/50

*	[0] gyro_x
*	[1] gyro_y
*	[2] gyro_z

