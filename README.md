
# ESP32 Flight Controller (ongoing)
Built using IDF 5.5.1, using custom implementation of DSHOT600, MPU6050 and CRFS parsing.

## Key Points
* 2x MPU6050 support
* BMP280
* CRSF
* DSHOT600 (rmt)

## Hardware
* ESP32 ttgo
* MPU6050
* BMP280
* Radiomaster RP3 ExpressLRS ELRS 2.4GHz
* Sequre E70
* GEPRC M1025 (not implemented yet)

## Performance
* IMU 200Hz (100Hz per imu)
* CRSF 500Hz
* control loop 2kHz 

## Components
* components/arduino/libraries/TFT_eSPI
* components/bmp280

## TODO
* Replace circular buffers with signal to improve performance
* Implement GPS module (GEPRC M1025)
* RPM filtering
* Antigravity Iterm
* Iterm relax
* Additional MultiWii support for DJI O4 Pro 
* Update readme with new board design
* angle/selflevel/acro modes

## Known issues
* ESP32 has issues reading voltages near min max values causing unreliable current readings from ESC.

### Schematics
# This is version 1 baord, version 2 currently in use
![image info](./images/dev_board.jpeg)

## noteable issues solved
* Stuttering motors - solved by sleep/wait after rmt write
* DMP above 100hz (write to fw) freaks out after a while
* Bidi dshot solved by circumventing rmt callback(took 80 us) with a custom one(takes 5 us). 


