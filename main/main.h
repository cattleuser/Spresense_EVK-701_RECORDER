/*
MIT License

Copyright (c) 2020 TechnoPro, Inc. TechnoPro Design Company

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _MAIN_H_
#define _MAIN_H_

/**
 * @file cow_log_sys.ino
 * @author TechnoPro, Inc. TechnoPro Design Company
 * @brief Save Animal acceleration and pressure to SD card.
 * @details It is a sketch for recording the behavior of the Animal.
 */

/**
 * @brief Includes <System Includes> , "Project Includes"
 */
#include <GNSS.h>
#include <GNSSPositionData.h>
#include <Wire.h>
#include <RTC.h>
#include <LowPower.h>
#include <Watchdog.h>
#include "gnss_tracker.h"
#include "gnss_nmea.h"
#include "gnss_file.h"
#include "KX122.h"
#include "BM1383AGLV.h"

/**
 * @brief Macro definitions
 */
/* Output settings */
#define NMEA_OUT_UART          0              /** true 1, false 0 */
#define NMEA_OUT_FILE          0              /** true 1, false 0 */
#define SENSOR_OUT_UART        0              /** true 1, false 0 */
#define SENSOR_OUT_FILE        1              /** true 1, false 0 */

#define UART_DEBUG_MESSAGE     PrintNone
#define CONFIG_FILE_NAME       "tracker.ini"  /**< Config file name */
#define CONFIG_FILE_SIZE       4096           /**< Config file size */
#define INDEX_FILE_NAME        "index.ini"    /**< Index file name */
#define INDEX_FILE_SIZE        16             /**< Index file size */

/* Buffer settings */
#define STRING_BUFFER_SIZE     128            /**< String buffer size */
#define NMEA_BUFFER_SIZE       128            /**< NMEA buffer size */
#define SENSOR_BUFFER_SIZE     128            /**< SENSOR buffer size */
#define OUTPUT_FILENAME_LEN    20             /**< Output file name length. */

/* Communication settings */
#define SERIAL_BAUDRATE        115200         /**< Serial baud rate. */
#define SEPARATOR              0x0A           /**< Separator */

/* Interval settings */
#define SENSOR_INTERVAL        20             /**< [ms] */
#define STORE_RECORDS_NUM      25             /**< 1 to 1024 */
#define FILE_INTERVAL          1800000        /**< [ms] */
#define GPS_INTERVAL           1000           /**< [ms] */
#define SENSORBUFF             STORE_RECORDS_NUM * STRING_BUFFER_SIZE + STRING_BUFFER_SIZE


/* GNSS CONFIG */
#define SATELLIT_ESYSTEM       eSatGpsGlonassQz1c /** ParamSat */
#define MY_TIMEZONE_IN_SECONDS (9 * 60 * 60)  /** JST[s] */
#define INTERVAL_SEC           1              /** true 1, false 0 */

/* LED debug settings */
#define LED_DEBUG_MODE         0              /** set 1 true, set 0 false */

/**
 * @enum LoopState
 * @brief device state
 */
enum LoopState
{
  eStateIdle,
  eStateRenewFile,
  eStateGnssNonFix,
  eStateSensor,
  eStateError,
  eStateWriteError
};

/**
 * @enum ParamSat
 * @brief Satellite system
 */
enum ParamSat
{
  eSatGps,            /**< GPS */
  eSatGlonass,        /**< GLONASS */
  eSatGpsSbas,        /**< GPS+SBAS */
  eSatGpsGlonass,     /**< GPS+Glonass */
  eSatGpsQz1c,        /**< GPS+QZSS_L1CA */
  eSatGpsGlonassQz1c, /**< GPS+Glonass+QZSS_L1CA */
  eSatGpsQz1cQz1S,    /**< GPS+QZSS_L1CA+QZSS_L1S */
};

/**
 * @struct ConfigParam
 * @brief Configuration parameters
 */
typedef struct
{
  ParamSat      SatelliteSystem;  /**< Satellite system(GPS/GLONASS/ALL). */
  boolean       NmeaOutUart;      /**< Output NMEA message to UART(TRUE/FALSE). */
  boolean       NmeaOutFile;      /**< Output NMEA message to file(TRUE/FALSE). */
  boolean       SensorOutUart;    /**< Output Sensor message to UART(TRUE/FALSE). */
  boolean       SensorOutFile;    /**< Output Sensor message to file(TRUE/FALSE). */
  boolean       PramOutUart;      /**< Output Param message to UART(TRUE/FALSE). */
  boolean       PramOutFile;      /**< Output Param message to file(TRUE/FALSE). */
  unsigned int  IntervalSec;      /**< Positioning interval sec(1-300). */
  SpPrintLevel  UartDebugMessage; /**< Uart debug message(NONE/ERROR/WARNING/INFO). */
} ConfigParam;

/**
 * @brief Exported global variables
 */

/**
 * @brief Exported global functions (to be accessed by other files)
 */

 #endif /* _MAIN_H_ */
