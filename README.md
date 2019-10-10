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
--gnss_file.h/gnss_file.cpp/gnss_nmea.h/gnss_nmea.cpp/gnss_tracker.h

* SPRESENSE-SENSOR-EVK-701(https://github.com/RohmSemiconductor/Arduino)  
The referenced sample is as follows.  
--BM1383AGLV.h/BM1383AGLV.cpp/KX122.h/KX122.cpp

* The code created for this project is as follows.  
--cow_log.h/sd_acc_press_gps.ino

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

# How to use
* 1.Set up SPRESENSE Arduino IDE according to (https://developer.sony.com/ja/develop/spresense/)
* 2.T.B.D
* 3.T.B.D
* 4.Data Format is XXXXX
* 5.

# Referense
* https:XXXXX/XXXXX
* TeraTerm
* T.B.D
