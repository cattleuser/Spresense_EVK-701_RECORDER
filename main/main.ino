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
 * @brief gloval variables
 */
ConfigParam Parameter = {};                   /**< Configuration parameters */
word state = eStateIdle;
SpGnss Gnss;                                  /**< SpGnss object */
unsigned long write_size = 0;

/**
 * @brief private variables
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
volatile static unsigned long time_interval_alive = 0;
volatile static unsigned long time_interval_file = 0;
volatile static unsigned long time_interval_gps = 0;          /**< to update gps  */
volatile static unsigned long time_interval_sensor = 0;       /**< to update buff */
volatile static unsigned long BuffSize = 0;
volatile static SpNavData NavData = {};
volatile static char SensorBuff[SENSORBUFF] = {};
volatile static int records_num = 0;

/**
 * @brief global APIs
 */
String getNmeaGga(SpNavData* pNavData);
void SetupPositioning(void);
void Led_isState(void);

/**
 * @brief private APIs
 */
static void Led_isAlive(void);
static void Led_AliveBlink(void);
static void UpdateFileNumber(void);
static String getSensor(void);
static void GpsProcessing(void);
static void SensorProcessing(void);
static void CheckFileRenew(void);
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

static void Led_AliveBlink(void)
{
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
}

static void CheckFileRenew(void)
{
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

  /* GPS PROCESSING. */
  time_interval_gps = time_current - time_past_gps;
  if(time_interval_gps >= GPS_INTERVAL)
  {
    time_past_gps = time_current; /*timer set.*/
    
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
          Serial.print(NmeaString.c_str());
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
  else
  {
    /* do nothing. */
  }
}

static void SensorProcessing(void)
{
  String SensorString = "";
  static int records_num = 0;

  time_interval_sensor = time_current - time_past_sensor;
  if(time_interval_sensor >= SENSOR_INTERVAL)
  {
    time_past_sensor = time_current;
    /* Buffer Clear */
    if(state_last == eStateGnssNonFix)
    {
      SensorBuff[0] = '\0';
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
        Serial.print(SensorString.c_str());
      }
      else
      {
        /* do nothing. */
      }
  
      if (Parameter.SensorOutFile == true)
      {
        records_num += 1;
        strncat(SensorBuff, SensorString.c_str(), strlen(SensorString.c_str()));

        /* Counter Check to Write. */
        if(records_num >= STORE_RECORDS_NUM)
        {
          if (SensorBuff[0] != '\0')
          {
            write_size = WriteSD(SensorBuff, strlen(SensorBuff));
            /* Check result. */
            if (write_size == strlen(SensorBuff))
            {
              records_num = 0;
              SensorBuff[0] = '\0'; 
            }
            else
            {
              state = eStateWriteError;
              Led_isState();
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
      else
      {
        /* do nothing. */
      }
    }
  }
  else
  {
    /* Do nothing. */
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
\ * @brief Activate GNSS device and Sensor modules.
 */
void setup(void)
{
  Watchdog.begin();
  Watchdog.start(20000);

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

  state = eStateRenewFile;
  Led_isState();
}

/**
 * @brief Main loop
 */
void loop(void)
{
  Watchdog.kick();
  time_current = millis();
  Led_AliveBlink();
  Led_isState();
  CheckFileRenew();

  switch(state)
  {
    case  eStateSensor:  /**< Loop is not activated */
      if(state != state_last)
      {
        Gnss.stop();
        Wire.begin();
        OpenSD(FileSensorTxt, (FILE_WRITE | O_APPEND));
      }
      else
      {
        /* do nothing. */
      }
      SensorProcessing();
      /* Task  */
      state_last = eStateSensor;
      break;

    case  eStateRenewFile:
      if(state != state_last)
      {
        CloseSD();
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
      state = eStateGnssNonFix;
      break;

    case  eStateGnssNonFix:
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
      state_last = eStateGnssNonFix;
      if(TimefixFlag == 1)
      {
        state = eStateSensor;
      }
      break;

    default:
      state = eStateError;
      Led_isState();
      break;
  }
}
