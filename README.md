# Overview
Available in Arduino development environment.
Data logger for Sony's IoT computer SPRESENSE.
There is a function to write data such as inertia sensor and time stamp to the SD card.
The time is corrected with the GPS signal.

**Required devices**
* SPRESENSE+CXD5602PWBEXT1(https://developer.sony.com/ja/develop/spresense/)
* SPRESENSE-SENSOR-EVK-701(https://www.rohm.co.jp/support/spresense-add-on-board)

**Source of code**
* spresense-arduino-compatible(https://github.com/sonydevworld/spresense-arduino-compatible)  
The referenced sample is as follows.  
-gnss_file.h/gnss_file.cpp/gnss_nmea.h/gnss_nmea.cpp/gnss_tracker.h

* SPRESENSE-SENSOR-EVK-701(https://github.com/RohmSemiconductor/Arduino)  
The referenced sample is as follows.  
-BM1383AGLV.h/BM1383AGLV.cpp/KX122.h/KX122.cpp

* The code created for this project is as follows.  
-cow_log.h/sd_acc_press_gps.ino

# Features
* A new file is created every 30 minutes.
* After creating a new file, correct the RTC time with the GPS signal before starting sensing.
* The sensing data is stored in the SD card slot of CXD5602PWBEXT1.
* Data logging will not start until the RTC time correction using the GPS signal is complete. An accurate record of the time remains.
* The recorded acceleration and pressure are 25 [Hz] intervals.
* The maximum acceleration to be acquired is ± 4 [G] and the resolution is 1 [mG].
* The unit of air pressure to be acquired is [hPa].
* Does not drive interrupts.

# Disclaimer
* This software is MIT license.
* We cannot respond to inquiries.

# State transition
The software performs the following state transitions: The LED corresponding to the status lights up.  

| State | Meaning | LED3 | LED2 | LED1 | LED0 |
|:---|:---|:---|:---|:---|:---|
| eStateIdle | System idle state | off | off | on | blinking |
| eStateSetup | Setup state | off | on | off | off |
| eStateRenewFile | File update status | off | on | on | blinking |
| eStateGnss | GPS time data correction status | on | off | off | blinking |
| eStateSensor | Sensor data acquisition status | on | off | on | blinking |
| eStateError | Error occurred | on | on | off | hold |
| eStateWriteError | Write error occurred | on | on | on | hold |

# Data format
The data stored on the SD card is in the following format.  

| flag | Terminal number | hh:mm:ss.ss | Serial number | interval[ms] | Acc-X[G] | Acc-Y[G] | Acc-Z[G] | Barometric pressure[hPa] |
|:---|:---|:---|:---|:---|:---|:---|:---|:---|

# How to use
1.Set up SPRESENSE Arduino IDE according to (https://developer.sony.com/ja/develop/spresense/)  
2.Connect CXD5602PWBEXT1 and SPRESENSE-SENSOR-EVK-701 to SPRESENSE.  
3.Connect the SD card to CXD5602PWBEXT1.  
4.When the power is turned on, the device starts up and logs are stored on the SD card.  
5.Turn off the power and remove the SD card.  

# Referense
* https:XXXXX/XXXXX
* TeraTerm
* T.B.D
