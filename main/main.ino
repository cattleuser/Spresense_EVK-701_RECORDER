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

/**
 * @file sd_acc_press_gps.ino
 * @author TechnoPro, Inc. TechnoPro Design Company
 * @brief Save Animal acceleration and pressure to SD card.
 * @details It is a sketch for recording the behavior of the Animal.
 */

/**
 * @brief <System Includes> , "Project Includes"
 */
#include "main.h"

/**
 * @brief Private gloval variables and functions
 */
ConfigParam Parameter = {};                   /**< Configuration parameters */
word state = eStateIdle;
SpGnss Gnss;                                  /**< SpGnss object */
unsigned long write_size = 0;
void Led_isState();

/**
 * @brief Private private variables and functions
 */
volatile static char rc = 0;/* flag */
volatile static char IndexData[INDEX_FILE_SIZE] = {};
volatile static char FileNmeaTxt[OUTPUT_FILENAME_LEN] = {};   /**< Output file name */
volatile static char FileSensorTxt[OUTPUT_FILENAME_LEN] = {}; /**< Output file name */
volatile static word led = 0;
volatile static word TimefixFlag = 0;
volatile static word state_last = eStateIdle;
volatile static int diff = 0;
volatile static int FileCount = 0;
volatile static int ReadSize = 0;
volatile static unsigned long seq = 0;                        /**< sequence no    */
volatile static unsigned long time_current = 0;               /**< to get current */
volatile static unsigned long time_past_alive = 0;
volatile static unsigned long time_past_file = 0;
volatile static unsigned long time_past_gps = 0;              /**< to update gps  */
volatile static unsigned long time_past_sensor = 0;           /**< to update buff */
volatile static unsigned long time_past_sensor_write = 0;     /**< to write file  */
volatile static unsigned long time_interval_alive = 0;
volatile static unsigned long time_interval_file = 0;
volatile static unsigned long time_interval_gps = 0;          /**< to update gps  */
volatile static unsigned long time_interval_sensor = 0;       /**< to update buff */
volatile static unsigned long time_interval_sensor_write = 0; /**< to write file  */
volatile static unsigned long BuffSize = 0;
volatile static SpNavData NavData = {};

/**
 * @brief global APIs
 */
void SetupPositioning(void);

/**
 * @brief private APIs
 */
static void Led_isAlive(void);
static void UpdateFileNumber(void);
static String getSensor(void);
static void GpsProcessing(void);
static void SensorProcessing(void);
static KX122 kx122(KX122_DEVICE_ADDRESS_1F); /**< acceleration */
static BM1383AGLV bm1383aglv;                /**< barometor */

/**
 * @brief Turn on / off the LED0 for CPU active notification.
 */
static void Led_isAlive(void)
{
  if(LED_DEBUG_MODE)
  {
    if (led == true)
    {
      led = false;
      ledOff(PIN_LED0);
    }
    else if(led == false)
    {
      led = true;
      ledOn(PIN_LED0);
    }
    else
    {
      state = eStateError;
      Led_isState();
    }
  }
  else
  {
    /* Do nothing */
  }
}

/**
\ * @brief Activate GNSS device and setup positioning
 */
void setup(void)
{
  Watchdog.begin();
  Watchdog.start(20000);

  state = eStateSetup;
  Led_isState();

  /* Set serial baudeate. */
  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial);

  LowPower.begin();
  LowPower.clockMode(CLOCK_MODE_156MHz);                  

  /* Initialize gps */
  SetupPositioning();

  /* Initialize acceleration */
  Wire.begin();
  rc = kx122.init();
  if (rc != 0)
  {
    state = eStateError;
    Led_isState();
  }
  else
  {
   /* do nothing. */
  }

  /* barometer & temperature */
  rc = bm1383aglv.init();
  if (rc != 0)
  {
    state = eStateError;
    Led_isState();
  }
  else
  {
    /* do nothing. */
  }

  state_last = eStateSetup;
  state = eStateRenewFile;
  Led_isState();
}

/**
 * @brief Get file number.
 * 
 * @return File count
 */
static void UpdateFileNumber(void)
{
  FileCount = 0;
  ReadSize = 0;
  IndexData[INDEX_FILE_SIZE] = {};
  FileNmeaTxt[0] = 0;
  FileSensorTxt[0] = 0;
  seq = 0;

  /* Open index file. */
  ReadSize = ReadChar(IndexData, INDEX_FILE_SIZE, INDEX_FILE_NAME, FILE_READ);
  if (ReadSize != 0)
  {
    /* Use index data. */
    FileCount = strtoul(IndexData, NULL, 10);
    FileCount++;

    Remove(INDEX_FILE_NAME);
  }
  else
  {
    /* Init file count. */
    FileCount = 1;
  }

  /* Update index.txt */
  snprintf(IndexData, sizeof(IndexData), "%08d", FileCount);
  write_size = WriteChar(IndexData, INDEX_FILE_NAME, FILE_WRITE);
  if (write_size != strlen(IndexData))
  {
    state = eStateWriteError;
  }
  else
  {
    /* do nothing. */
  }

  if (Parameter.NmeaOutFile == true)
  {
    /* Create a file name to store NMEA data. */
    snprintf(FileNmeaTxt, sizeof(FileNmeaTxt), "NMEA%08d.CSV", FileCount);
  }
  else
  {
    /* do nothing. */
  }

  if (Parameter.SensorOutFile == true)
  {
    /* Create a file name to store SENSOR data. */
    snprintf(FileSensorTxt, sizeof(FileSensorTxt), "SENSOR%08d.CSV", FileCount);
  }
  else
  {
    /* do nothing. */
  }
}

static void GpsProcessing(void)
{
  diff = 0;
  static char *pNmeaBuff     = NULL;
  String NmeaString = "";

  /* Get NavData. */
  Gnss.getNavData(&NavData);
  SpGnssTime *time = &NavData.time;

  /* check if time update */
  if(Gnss.waitUpdate() && (NavData.posFixMode >= 1) && (time->year >= 2000))
  {
    /* If device can get the value of gps correctlv. */
    /* Check if the acquired UTC time is accurate. */
    /* get and evacuation RTC time to "now" */
    RtcTime now = RTC.getTime();
  
    /* Convert SpGnssTime to RtcTime */
    RtcTime gps(time->year, time->month, time->day, time->hour, time->minute, time->sec, time->usec * 1000);

    gps += MY_TIMEZONE_IN_SECONDS;

    /* Set the time difference */
    diff = now - gps;
    if (abs(diff) >= 1)
    {
      /* set GPS time to RTC. */
      RTC.setTime(gps);
    }
    else
    {
      /* Judged that time was corrected. */
      TimefixFlag = 1;
    }

    /* Get Nmea Data. */
    NmeaString = getNmeaGga(&NavData);

    if (strlen(NmeaString.c_str()) == 0)
    {
      state = eStateError;
      Led_isState();
    }
    else
    {
      /* Output Nmea Data. */
      if (Parameter.NmeaOutUart == true)
      {
        /* To Uart. */
        APP_PRINT(NmeaString.c_str());
      }
      else
      {
        /* do nothing. */
      }
    }

    if (Parameter.NmeaOutFile == true)
    {
      /* To SDCard. */
      BuffSize = NMEA_BUFFER_SIZE * 4;

      if (pNmeaBuff == NULL)
      {
        /* Alloc buffer. */
        pNmeaBuff = (char*)malloc(BuffSize);
        if (pNmeaBuff != NULL)
        {
          /* Clear Buffer */
          pNmeaBuff[0] = 0x00;
        }
        else
        {
          /* do nothing. */
        }
      }
      else
      {
        /* do nothing. */
      }

      if (pNmeaBuff != NULL)
      {
        /* Store Nmea Data to buffer. */
        strncat(pNmeaBuff, NmeaString.c_str(), BuffSize);
      }
      else
      {
        /* do nothing. */
      }

      /* Check Sensor buffer. */
      if(strlen(pNmeaBuff) > (BuffSize - NMEA_BUFFER_SIZE))
      {
        state = eStateError;
        Led_isState();
      }
      else
      {
        /* do nothing. */
      }
    
      if (pNmeaBuff != NULL)
      {
        /* Write Nmea Data. */
        write_size = WriteChar(pNmeaBuff, FileNmeaTxt, (FILE_WRITE | O_APPEND));
        /* Check result. */
        if (write_size != strlen(pNmeaBuff))
        {
          while(1);
        }
        else
        {
          /* do nothing. */
        }
    
        /* Clear Buffer */
        pNmeaBuff[0] = 0x00;
      }
      else
      {
        /* do nothing. */
      }
    }
    else
    {
      /* do nothing. */
    }
  }
  else
  {
    /* do nothing. */
  }
}

static void SensorProcessing(void)
{
  static char* pSensorBuff = NULL;
  String SensorString = "";

  /* Buffer Clear */
  if(state_last == eStateGnss)
  {
    pSensorBuff[0] = 0x00;
  }
  else
  {
    /* Do nothing. */
  }

  /* Get senser data here. */
  SensorString = getSensor();

  if (strlen(SensorString.c_str()) == 0)
  {
    state = eStateError;
    Led_isState();
  }
  else
  {
    /* Output Sensor Data. */
    if (Parameter.SensorOutUart == true)
    {
      /* To Uart. */
      APP_PRINT(SensorString.c_str());
    }
    else
    {
      /* do nothing. */
    }

    if (Parameter.SensorOutFile == true)
    {
      /* To SDCard. */
      BuffSize = SENSOR_BUFFER_SIZE * (SENSOR_INTERVAL_WRITE / SENSOR_INTERVAL);

      if (pSensorBuff == NULL)
      {
        /* Alloc buffer. */
        pSensorBuff = (char*)malloc(BuffSize);
        if (pSensorBuff != NULL)
        {
          /* Clear Buffer */
          pSensorBuff[0] = 0x00;
        }
        else
        {
          /* do nothing. */
        }
      }
      else
      {
        /* do nothing. */
      }

      if (pSensorBuff != NULL)
      {
        /* Store Sensor Data to buffer. */
        strncat(pSensorBuff, SensorString.c_str(), BuffSize);
      }
      else
      {
        /* do nothing. */
      }
    }
    else
    {
      /* do nothing. */
    }

    /* Check Sensor buffer. */
    if(strlen(pSensorBuff) > (BuffSize - SENSOR_BUFFER_SIZE))
    {
      state = eStateError;
      Led_isState();
    }
    else
    {
      /* do nothing. */
    }
  }

  /* SENSER WRITE PROCESSING. */
  time_interval_sensor_write = time_current - time_past_sensor_write;
  /* Counter Check to Write. */
  if(time_interval_sensor_write >= SENSOR_INTERVAL_WRITE)
  {
    time_past_sensor_write = time_current;

    if (pSensorBuff != NULL)
    {
      /* Write Sensor Data. */
      write_size = WriteChar(pSensorBuff, FileSensorTxt, (FILE_WRITE | O_APPEND));
      /* Check result. */
      if (write_size != strlen(pSensorBuff))
      {
        state = eStateWriteError;
        Led_isState();
      }
      else
      {
        /* do nothing. */
      }
  
      /* Clear Buffer */
      pSensorBuff[0] = 0x00;
    }
    else
    {
      /* do nothing .*/
    }
  }
  else
  {
    /* do nothing. */
  }
}

static String getSensor(void)
{
  rc = 0;/* flag */
  String Sensor = "";
  char StringBuffer[STRING_BUFFER_SIZE] = {};
  float acc[3];/* acceleration */
  float barom = 0, temp = 0;

  /* acceleration */
  rc = kx122.get_val(acc);
  if (rc != 0)
  {
    Serial.println("KX122 failed.");
  }

  /* barometer & no temperature */
  rc = bm1383aglv.get_val(&barom, &temp);
  if (rc != 0)
  {
    Serial.println("BM1383AGLV failed.");
  }

  /* Set Header. */
  Sensor = "$V00300,";/* sign name */
  Sensor += "0x0001,";/* device no */

  RtcTime now = RTC.getTime();

  /* Time when rtc was modified by gps. */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d %02d:%02d:%02d.%03d,", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), now.nsec() / 1000000);
  Sensor += StringBuffer;

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%d,", seq++);/* sequence no */
  Sensor += StringBuffer;

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%d,", time_interval_sensor);/* elapsed time */
  Sensor += StringBuffer;

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%5.3f,", acc[0]);/* acceleration (X) */
  Sensor += StringBuffer;

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%5.3f,", acc[1]);/* acceleration (Y) */
  Sensor += StringBuffer;

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%5.3f,", acc[2]);/* acceleration (Z) */
  Sensor += StringBuffer;

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%4.4f",barom);/* barometer */
  Sensor += StringBuffer;

  Sensor += "\n";

  return Sensor;
}

/**
 * @brief GNSS tracker loop
 * 
 * @details Positioning is performed for the first 300 seconds after setup.
 *          After that, in each loop processing, it sleeps for SleepSec 
 *          seconds and performs positioning ActiveSec seconds. 
 *          The gnss_tracker use SatelliteSystem sattelites for positioning.\n\n
 *  
 *          Positioning result is notificated in every IntervalSec second.
 *          The result formatted to NMEA will be saved on SD card if the 
 *          parameter NmeaOutFile is TRUE, or/and output to UART if the 
 *          parameter NmeaOutUart is TRUE. NMEA is buffered for each 
 *          notification. Write at once when ActiveSec completes. If SleepSec 
 *          is set to 0, positioning is performed continuously.
 */
void loop(void)
{
  Watchdog.kick();

  /* Check state. */
  time_current = millis();

  time_interval_alive = time_current - time_past_alive;
  if(time_interval_alive >= 1000)
  {
    time_past_alive = time_current;
    Led_isAlive();
  }
  else
  {
    /* do nothing. */
  }

  switch(state)
  {
    case  eStateIdle:
      /* dont't update state */
      /* state_last = eStateIdle; */
      state = eStateIdle;
      break;

    case  eStateSetup:
      Led_isState();
      state_last = eStateSetup;
      state = eStateIdle;
      break;

    case  eStateRenewFile:
      Led_isState();
      if(state != state_last)
      {
        TimefixFlag = 0;
        RTC.end();
        Gnss.stop();
        Wire.end();
      }
      else
      {
        /* do nothing. */
      }
      UpdateFileNumber();
      state_last = eStateRenewFile;
      state = eStateIdle;
      break;

    case  eStateGnss:
      Led_isState();
      if(state != state_last)
      {
        Wire.end();
        Gnss.start(HOT_START);
        RTC.begin();
      }
      else
      {
        /* do nothing. */
      }
      GpsProcessing();
      state_last = eStateGnss;
      state = eStateIdle;
      break;

    case  eStateSensor:  /**< Loop is not activated */
      Led_isState();
      if(state != state_last)
      {
        Gnss.stop();
        Wire.begin();
      }
      else
      {
        /* do nothing. */
      }
      SensorProcessing();
      state_last = eStateSensor;
      state = eStateIdle;
      break;

    default:
      state = eStateError;
      Led_isState();
      break;
  }

  /* RENEW FILE */
  time_interval_file = time_current - time_past_file;
  if(time_interval_file >= FILE_INTERVAL)
  {
    time_past_file = time_current;
    state = eStateRenewFile;
  }
  else
  {
    /* do nothing. */
  }

  if(TimefixFlag == 1)
  {
    /* SENSOR PROCESSING. */
    time_interval_sensor = time_current - time_past_sensor;
    if(time_interval_sensor >= SENSOR_INTERVAL)
    {
      time_past_sensor = time_current;
      state = eStateSensor;
    }
    else
    {
      /* do nothing. */
    }
  }
  else
  { 
    /* GPS PROCESSING. */
    time_interval_gps = time_current - time_past_gps;
    if(time_interval_gps >= GPS_INTERVAL)
    {
      time_past_gps = time_current;
      state = eStateGnss;
    }
    else
    {
      /* do nothing. */
    }
  }
}
