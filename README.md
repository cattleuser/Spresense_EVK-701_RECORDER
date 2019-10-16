# Overview
Available in Arduino development environment.
Data logger for Sony's IoT development board SPRESENSE.
The function of this data logger is to write inertia sensor data along with time stamp. Time stamp data is corrected with GPS signal.

# Features
* Compatibility with QZSS Michibiki.
* Acceleratia and pressure data are recorded with 40[ms] time intervals.
* The acceleration range is ± 4 [G] and the resolution is 1 [mG].
* The unit of air pressure resolution is 1 [hPa].
* After creating a new file, RTC is corrected with the GPS signal before data logging.
* Data logging will not start until the RTC is corrected using the GPS signal.
* When the time is corrected, the GPS reception process stops. The GPS reception process will sleep until the next time recording remains accurate within adjustments.
* The data is stored in the SD card slot of CXD5602PWBEXT1.
* A new file is created every 30 minutes.
* Does not drive interrupts.

# Quasi-Zenith Satellite Orbit
QZSS, which complements GPS, is a system especially for usage in the Asia-Oceania regions, with a focus on Japan.

![isos7j00000003du](https://user-images.githubusercontent.com/49668780/66568620-ca238480-eba4-11e9-9387-fcfcd4ab10a6.jpg)  
Source : https://qzss.go.jp/en/technical/technology/orbit.html

# Disclaimer
* This software is MIT license.
* We cannot respond to inquiries.

## License
MIT

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

| flag(*1) | Terminal number(*2) | hh:mm:ss.ss | Serial number | interval[ms] | Acc-X[G] | Acc-Y[G] | Acc-Z[G] | Barometric pressure[hPa] |
|:---|:---|:---|:---|:---|:---|:---|:---|:---|

(*1)The meaning of the record. You can edit here.  
(*2)Used when operating multiple terminals at the same time. You can edit here.  

# Requirements
**devices**
* SPRESENSE+CXD5602PWBEXT1  
https://developer.sony.com/ja/develop/spresense/
* SPRESENSE-SENSOR-EVK-701  
https://www.rohm.co.jp/support/spresense-add-on-board

**Source of code**
* spresense-arduino-compatible  
https://github.com/sonydevworld/spresense-arduino-compatible  
The referenced sample is as follows.  
./Arduino15/packages/SPRESENSE/hardware/spresense/1.0.0/libraries/GNSS/examples/gnss_tracker  
  * /gnss_file.h/gnss_file.cpp/gnss_nmea.h/gnss_nmea.cpp/gnss_tracker.h

* SPRESENSE-SENSOR-EVK-701  
https://github.com/RohmSemiconductor/Arduino  
The referenced sample is as follows.  
  * /BM1383AGLV.h/BM1383AGLV.cpp/KX122.h/KX122.cpp

* The code created for this project is as follows.  
  * /cow_log.h/sd_acc_press_gps.ino

# How to use
1. Set up SPRESENSE Arduino IDE is as follows.  
https://developer.sony.com/ja/develop/spresense/  
1. Connect CXD5602PWBEXT1 and SPRESENSE-SENSOR-EVK-701 to SPRESENSE.  
1. Connect the SD card to CXD5602PWBEXT1.  
1. When the power is turned on, the device starts up, the time is corrected by GPS, and the log is saved on the SD card.  
1. Turn off the power and remove the SD card.  

# Reference website
* Try Spresense's GNSS (GPS) reception function  
https://y2lab.org/blog/gudget/trying-gnss-receiving-function-on-spresence-7497/
* Play with RaspBerry Pi  
http://nopnop2002.webcrow.jp/

# Tools
* Tera Term Home Page  
https://ttssh2.osdn.jp/
