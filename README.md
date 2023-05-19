# Tray Object Detection

Tray Object Detection using STM's ToF sensor, VL53L7CX

## Table of Contents

- [How to compile](#How_to_compile)
- [Usage](#usage)
- [Comment](#comment)

## How to compile

g++ -std=c++14 -I ./src -I (boost include directory) -o a.out ToFSensor.cpp TrayObjDetection.cpp

<br>
in case of macOS with latest boost installed by brew:<br>
(boost include directory) = /opt/homebrew/Cellar/boost/1.81.0_1/include<br>
- boost is used for calculating critical values for t-test, which might be obtained through a t-table eventually removing the need for this lib.

## Usage

run "./a.out" (when output file name was set as a.out)

press one of the following keys to do what you want:

    q: quit this program
    0: zero adjustment
    d: run object detectection on the tray
    s: start or stop generating and inserting sensor data for simulation
    h: display this help message
    
## Comment

Things yet to be done:
- 나중에 적을 예정 
