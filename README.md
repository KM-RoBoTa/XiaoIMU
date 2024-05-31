# IMU: Seeed Studio XIAO nRF52840 Sense
This repository contains all the necessary code to use Seeed Studio XIAO nRF52840 Sense's IMU. <br /> 
You can find here:
- the Arduino sketch to be flashed onto the Xiao board to continuously read the IMU and send the data through a serial communication, found in the ```IMU_arduinoSketch``` folder
- the C++ code for the Linux OS that reads the data sent by the Xiao board, found in the ```readIMU_cppCode```
- an example of how to use the code, found in the ```example``` folder

## Xiao board's Arduino sketch
The used board is the Seeed XIAO BLE Sense - nRF52840, found in the ```Seeed nRF52840 mbed-enabled boards``` board package, which can be found here: https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json

The sketch also uses the [Seeed_Arduino_LSM6DS3](https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3) library, which needs to be added as a .zip library to the sketch.

Some useful links:
- IMU usage: https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/

## C++ code to read the data
To use the code, you simply need to add the 2 files in your project, and include the ```xiao_imu.hpp``` header. 

You need to create an instance of the ```IMU``` class, which automatically opens the port, creates a thread and starts continuously reading the incoming data. <br /> 
That data can be fetched by the main thread using the ```IMU::getValues()``` method. 

The acceleration values are in [m/s²], while the angular velocities in [deg/s]. You can find the drawing of the IMU's axes in the top folder of this repository.

**Speed notes** <br /> 
The code is very quick, however the data sending is very slow (a packet received every 7ms). <br /> 
By default, in Linux, the USB-serial communication is very slow. <br />  
If using an FTDI serial-to-USB converter (assigned a /dev/ttyUSBx port), one can set the ASYNC_LOW_LATENCY flag to the serial structure, which makes the communication *much* faster.  <br /> 
Unfortunately, it appears we do not have an FTDI converter (the Xiao board gets assigned a /dev/ttyACMx port), and the flag is unusable, leaving us stuck with the slow communication.

Additional personal note: As I am not an expert in serial communication, I would be *very* happy to be proven wrong if there is a way to make it faster than the current crawl :snail:

## Example
The example is very basic to showcase the use of the code: the main thread fetches and prints the IMU values from the IMU's thread every 10ms. 

To use the example, first plug in the IMU and check the assigned port. If the port is different than "/dev/ttyACM0", edit the main.cpp. <br /> 
To build the example: 
```bash
cd <path-to-the-repo-folder>/example
mkdir build && cd build 
cmake ../
make imu
```
To run the code: ```./imu``` from the build folder.