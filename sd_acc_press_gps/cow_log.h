/*
MIT License

Copyright (c) 2019 Animal Silent Voice Project

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

/**
 * @file cow_log_sys.ino
 * @author Technopro
 * @brief Save Animal acceleration and pressure to SD card.
 * @details It is a sketch for recording the behavior of the Animal.
 */

/**
 * @brief Includes <System Includes> , "Project Includes"
 */

/**
 * @brief Macro definitions
 */
#define CONFIG_FILE_NAME       "tracker.ini"  /**< Config file name */
#define CONFIG_FILE_SIZE       4096           /**< Config file size */
#define INDEX_FILE_NAME        "index.ini"    /**< Index file name */
#define INDEX_FILE_SIZE        16             /**< Index file size */
#define STRING_BUFFER_SIZE     128            /**< String buffer size */
#define NMEA_BUFFER_SIZE       128            /**< NMEA buffer size */
#define SENSOR_BUFFER_SIZE     128            /**< SENSOR buffer size */
#define OUTPUT_FILENAME_LEN    20             /**< Output file name length. */
#define DEFAULT_INTERVAL_SEC   1              /**< Default positioning interval in seconds. */
#define SERIAL_BAUDRATE        115200         /**< Serial baud rate. */
#define SEPARATOR              0x0A           /**< Separator */
#define SENSOR_INTERVAL        40             /**< [ms] */
#define SENSOR_INTERVAL_WRITE  4000           /**< [ms] */
#define FILE_INTERVAL          1800000        /**< [ms] */
#define GPS_INTERVAL           1000           /** [ms] */
#define MY_TIMEZONE_IN_SECONDS (9 * 60 * 60)  /** JST[s] */

/**
 * @enum LoopState
 * @brief device state
 */
enum LoopState
{
  eStateIdle,
  eStateSetup,
  eStateRenewFile,
  eStateGnss,
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
  boolean       SensorOutUart;    /**< Output NMEA message to UART(TRUE/FALSE). */
  boolean       SensorOutFile;    /**< Output NMEA message to file(TRUE/FALSE). */
  unsigned int  IntervalSec;      /**< Positioning interval sec(1-300). */
  SpPrintLevel  UartDebugMessage; /**< Uart debug message(NONE/ERROR/WARNING/INFO). */
} ConfigParam;

/**
 * @brief Exported global variables
 */

/**
 * @brief Exported global functions (to be accessed by other files)
 */
 KX122 kx122(KX122_DEVICE_ADDRESS_1F); /**< acceleration */
 BM1383AGLV bm1383aglv;                /**< barometor */
