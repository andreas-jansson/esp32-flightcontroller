
# ESP32 Flight Controller (ongoing)


## Key Points
* 2x MPU6050 
* BMP280
* CRSF
* DSHOT600 (rmt)

## Hardware
* ESP32 ttgo
* MPU6050
* BMP280
* Radiomaster RP3 ExpressLRS ELRS 2.4GHz
* Sequre E70

## Performance
* IMU 200Hz (100Hz per imu)
* CRSF 500Hz
* control loop 2,5kHz 

## Build Info
* Built using IDF 5.1.1


## Components
* components/arduino/libraries/TFT_eSPI
* components/bmp280

## TODO
* Replace circular buffers with signal to improve performance
* Implement additional telemetry, drone -> controller
* Implement GPS module