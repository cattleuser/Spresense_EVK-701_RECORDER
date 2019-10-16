/*
MIT License

Copyright (c) 2019 TechnoPro, Inc. TechnoPro Design Company

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
#include <GNSS.h>
#include <GNSSPositionData.h>
#include <Wire.h>
#include <RTC.h>
#include <LowPower.h>
#include "gnss_tracker.h"
#include "gnss_nmea.h"
#include "gnss_file.h"
#include "KX122.h"
#include "BM1383AGLV.h"
#include "cow_log.h"

/**
 * @brief Private global variables and functions
 */
static word led = 0;
static word TimefixFlag = 0;
static word state = eStateIdle;
static word state_last = eStateIdle;
static unsigned long seq = 0;                        /**< sequence no    */
static unsigned long time_current = 0;               /**< to get current */

static unsigned long time_past_alive = 0;
static unsigned long time_past_file = 0;
static unsigned long time_past_gps = 0;              /**< to update gps  */
static unsigned long time_past_sensor = 0;           /**< to update buff */
static unsigned long time_past_sensor_write = 0;     /**< to write file  */

static unsigned long time_interval_alive = 0;
static unsigned long time_interval_file = 0;
static unsigned long time_interval_gps = 0;          /**< to update gps  */
static unsigned long time_interval_sensor = 0;       /**< to update buff */
static unsigned long time_interval_sensor_write = 0; /**< to write file  */

static unsigned long time_counter = 0;
static unsigned long BuffSize = 0;
static unsigned long write_size = 0;
static char FileNmeaTxt[OUTPUT_FILENAME_LEN] = {};   /**< Output file name */
static char FileSensorTxt[OUTPUT_FILENAME_LEN] = {}; /**< Output file name */
static char StringBuffer[STRING_BUFFER_SIZE] = {};
static SpNavData NavData = {};
static SpGnss Gnss;                                  /**< SpGnss object */
static ConfigParam Parameter = {};                   /**< Configuration parameters */
static String NmeaString = "";

/**
 * @brief APIs
 */
static int ParamCompare(const char *Input , const char *Refer);
static void Led_isActive(void);
static String MakeParameterString(ConfigParam *pConfigParam);
static int ReadParameter(ConfigParam *pConfigParam);
static void WriteParameter(ConfigParam *pConfigParam);
static int SetupParameter(void);
static unsigned long UpdateFileNumber(void);
static void SetupPositioning(void);
static String getSensor(void);
static void GpsProcessing(void);
static void SensorProcessing(void);

/* ############################################################################################################### */
/* ##                                       LED STATE                                                           ## */
/* ############################################################################################################### */

/**
 * @brief Turn on / off the LED0 for CPU active notification.
 */
static void Led_isActive(void)
{
  if (led == true)
  {
    led = false;
    ledOn(PIN_LED0);
  }
  else if(led == false)
  {
    led = true;
    ledOff(PIN_LED0);
  }
  else
  {
    state = eStateError;
    Led_isState();
  }
}

static void Led_isState()
{
  switch(state)
  {
    case  eStateIdle:
      ledOn(PIN_LED1);
      ledOff(PIN_LED2);
      ledOff(PIN_LED3);
      break;

    case eStateSetup:
      ledOff(PIN_LED1);
      ledOn(PIN_LED2);
      ledOff(PIN_LED3);
      break;

    case eStateRenewFile:
      ledOn(PIN_LED1);
      ledOn(PIN_LED2);
      ledOff(PIN_LED3);
      break;

    case eStateGnss:
      ledOff(PIN_LED1);
      ledOff(PIN_LED2);
      ledOn(PIN_LED3);
      break;

    case eStateSensor:
      ledOn(PIN_LED1);
      ledOff(PIN_LED2);
      ledOn(PIN_LED3);
      break;

    case eStateError:
      ledOff(PIN_LED1);
      ledOn(PIN_LED2);
      ledOn(PIN_LED3);
      while(1);
      break;

    case eStateWriteError:
      ledOn(PIN_LED1);
      ledOn(PIN_LED2);
      ledOn(PIN_LED3);
      while(1);
      break;

    default:
      ledOn(PIN_LED1);
      ledOn(PIN_LED2);
      ledOn(PIN_LED3);
      while(1);
      break;
  }
}

/* ############################################################################################################### */
/* ##                                       SETUP PROCESS                                                       ## */
/* ############################################################################################################### */
/**
 * @brief Activate GNSS device and setup positioning
 */
void setup(void)
{
  state = eStateSetup;
  Led_isState();
  /* put your setup code here, to run once: */
  byte rc;

  /* Set serial baudeate. */
  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial);

  /* Initialize RTC */
  RTC.begin();

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
 * @brief Setup configuration parameters.
 * 
 * @return 0 if success, -1 if failure
 */
static int SetupParameter(void)
{
  int ret;
  String ParamString;

  /* Read parameter file. */
  ret = ReadParameter(&Parameter);
  if (ret != OK)
  {
    /* If there is no parameter file, create a new one. */
    WriteParameter(&Parameter);
  }
  else
  {
    /* do nothing. */
  }

  /* Print parameter. */
  ParamString = MakeParameterString(&Parameter);
  APP_PRINT(ParamString.c_str());
  APP_PRINT("\n\n");

  return ret;
}

/**
 * @brief Read the ini file and set it as a parameter.
 * 
 * @details If there is no description, it will be the default value.
 * @param [out] pConfigParam Configuration parameters
 * @return 0 if success, -1 if failure
 */
static int ReadParameter(ConfigParam *pConfigParam)
{
  char *pReadBuff = NULL;
  int ReadSize = 0;
  int CharCount;
  int LineCount = 0;
  boolean FindSeparator = true;
  char *LineList[128] = {0,};
  int MaxLine = LineCount;
  char *pParamName;
  char *pParamData;
  int length;
  int tmp;

  pReadBuff = (char*)malloc(CONFIG_FILE_SIZE);
  if (pReadBuff == NULL)
  {
    APP_PRINT_E("alloc error:");
    APP_PRINT_E(CONFIG_FILE_NAME);
    APP_PRINT_E("\n");

    return -1;
  }
  else
  {
    /* do nothing. */  
  }

  /* Read file. */
  ReadSize = ReadChar(pReadBuff, CONFIG_FILE_SIZE, CONFIG_FILE_NAME, FILE_READ);
  if (ReadSize == 0)
  {
    APP_PRINT_E("read error:");
    APP_PRINT_E(CONFIG_FILE_NAME);
    APP_PRINT_E("\n");

    return -1;
  }
  else
  {
    /* do nothing. */
  }

  /* Set NULL at EOF. */
  pReadBuff[ReadSize] = NULL;

  /* Record the start position for each line. */
  for (CharCount = 0; CharCount < ReadSize; CharCount++)
  {
    if (FindSeparator == true)
    {
      LineList[LineCount] = &pReadBuff[CharCount];
      FindSeparator = false;
      LineCount++;
      if (LineCount >= 128)
      {
        break;
      }
      else
      {
        /* nop */
      }
    }
    else
    {
      /* do nothing. */
    }

    if (pReadBuff[CharCount] == SEPARATOR)
    {
      FindSeparator = true;
      pReadBuff[CharCount] = NULL;
    }
    else
    {
      /* do nothing. */
    }
  }

  /* Parse each line. */
  for (LineCount = 0; LineCount < MaxLine; LineCount++)
  {
    pParamName = LineList[LineCount];

    pParamData = NULL;
    if (pParamName[0] != ';')
    {
      length = strlen(pParamName);

      for (CharCount = 0; CharCount < length; CharCount++)
      {
        if (pParamName[CharCount] == '=')
        {
          pParamData = &(pParamName[CharCount + 1]);
          break;
        }
      }
    }

    /* Parse start. */
    if (pParamData == NULL)
    {
      /* nop */
    }
    else if (!ParamCompare(pParamName, "SatelliteSystem="))
    {
      if (!ParamCompare(pParamData, "GPS+GLONASS+QZSS_L1CA"))
      {
        pConfigParam->SatelliteSystem = eSatGpsGlonassQz1c;
      }
      else if (!ParamCompare(pParamData, "GPS+QZSS_L1CA+QZSS_L1S"))
      {
        pConfigParam->SatelliteSystem = eSatGpsQz1cQz1S;
      }
      else if (!ParamCompare(pParamData, "GPS+QZSS_L1CA"))
      {
        pConfigParam->SatelliteSystem = eSatGpsQz1c;
      }
      else if (!ParamCompare(pParamData, "GPS+GLONASS"))
      {
        pConfigParam->SatelliteSystem = eSatGpsGlonass;
      }
      else if (!ParamCompare(pParamData, "GLONASS"))
      {
        pConfigParam->SatelliteSystem = eSatGlonass;
      }
      else if (!ParamCompare(pParamData, "GPS+SBAS"))
      {
        pConfigParam->SatelliteSystem = eSatGpsSbas;
      }
      else if (!ParamCompare(pParamData, "GPS"))
      {
        pConfigParam->SatelliteSystem = eSatGps;
      }
      else
      {
        pConfigParam->SatelliteSystem = eSatGpsGlonassQz1c;
      }
    }
    else if (!ParamCompare(pParamName, "NmeaOutUart="))
    {
      if (!ParamCompare(pParamData, "FALSE"))
      {
        pConfigParam->NmeaOutUart = false;
      }
      else
      {
        pConfigParam->NmeaOutUart = true;
      }
    }
    else if (!ParamCompare(pParamName, "NmeaOutFile="))
    {
      if (!ParamCompare(pParamData, "FALSE"))
      {
        pConfigParam->NmeaOutFile = false;
      }
      else
      {
        pConfigParam->NmeaOutFile = true;
      }
    }
    else if (!ParamCompare(pParamName, "IntervalSec="))
    {
      tmp = strtoul(pParamData, NULL, 10);
      pConfigParam->IntervalSec = max(1, min(tmp, 300));
    }
    else if (!ParamCompare(pParamName, "UartDebugMessage="))
    {
      if (!ParamCompare(pParamData, "NONE"))
      {
        pConfigParam->UartDebugMessage = PrintNone;
      }
      else if (!ParamCompare(pParamData, "ERROR"))
      {
        pConfigParam->UartDebugMessage = PrintError;
      }
      else if (!ParamCompare(pParamData, "WARNING"))
      {
        pConfigParam->UartDebugMessage = PrintWarning;
      }
      else if (!ParamCompare(pParamData, "INFO"))
      {
        pConfigParam->UartDebugMessage = PrintInfo;
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
  return OK;
}

/**
 * @brief Compare parameter.
 * 
 * @param [in] Input Parameter to compare
 * @param [in] Refer Reference parameter
 * @return 0 if equal
 */
static int ParamCompare(const char *Input , const char *Refer)
{
  /* Set argument. */
  String InputStr = Input;
  String ReferStr = Refer;

  /* Convert to upper case. */
  InputStr.toUpperCase();
  ReferStr.toUpperCase();

  /* Compare. */
  return memcmp(InputStr.c_str(), ReferStr.c_str(), strlen(ReferStr.c_str()));
}

/**
 * @brief Create an ini file based on the current parameters.
 * 
 * @param [in] pConfigParam Configuration parameters
 * @return 0 if success, -1 if failure
 */
static void WriteParameter(ConfigParam *pConfigParam)
{
  String ParamString;

  /* Make parameter data. */
  ParamString = MakeParameterString(pConfigParam);

  /* Write parameter data. */
  if (strlen(ParamString.c_str()) != 0)
  {
    write_size = WriteChar(ParamString.c_str(), CONFIG_FILE_NAME, FILE_WRITE);
    if (write_size != strlen(ParamString.c_str()))
    {
      state = eStateWriteError;
      Led_isState();
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

/**
 * @brief Convert configuration parameters to String
 * 
 * @param [in] pConfigParam Configuration parameters
 * @return Configuration parameters as String
 */
static String MakeParameterString(ConfigParam *pConfigParam)
{
  const char *pComment;
  const char *pParam;
  const char *pData;
  char StringBuffer[STRING_BUFFER_SIZE];
  String ParamString;

  /* Set SatelliteSystem. */
  pComment = "; Satellite system(GPS/GLONASS/SBAS/QZSS_L1CA/QZSS_L1S)";
  pParam = "SatelliteSystem=";
  switch (pConfigParam->SatelliteSystem)
  {
    case eSatGps:
      pData = "GPS";
      break;

    case eSatGpsSbas:
      pData = "GPS+SBAS";
      break;

    case eSatGlonass:
      pData = "GLONASS";
      break;

    case eSatGpsGlonass:
      pData = "GPS+GLONASS";
      break;

    case eSatGpsQz1c:
      pData = "GPS+QZSS_L1CA";
      break;

    case eSatGpsQz1cQz1S:
      pData = "GPS+QZSS_L1CA+QZSS_L1S";
      break;

    case eSatGpsGlonassQz1c:
    default:
      pData = "GPS+GLONASS+QZSS_L1CA";
      break;
  }
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%s\n%s%s\n", pComment, pParam, pData);
  ParamString += StringBuffer;

  /* Set NmeaOutUart. */
  pComment = "; Output NMEA message to UART(TRUE/FALSE)";
  pParam = "NmeaOutUart=";
  if (pConfigParam->NmeaOutUart == FALSE)
  {
    pData = "FALSE";
  }
  else
  {
    pData = "TRUE";
  }
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%s\n%s%s\n", pComment, pParam, pData);
  ParamString += StringBuffer;

  /* Set NmeaOutFile. */
  pComment = "; Output NMEA message to file(TRUE/FALSE)";
  pParam = "NmeaOutFile=";
  if (pConfigParam->NmeaOutFile == FALSE)
  {
    pData = "FALSE";
  }
  else
  {
    pData = "TRUE";
  }
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%s\n%s%s\n", pComment, pParam, pData);
  ParamString += StringBuffer;

  /* Set SensorOutUart. */
  pComment = "; Output Sensor message to UART(TRUE/FALSE)";
  pParam = "SensorOutUart=";
  if (pConfigParam->SensorOutUart == FALSE)
  {
    pData = "FALSE";
  }
  else
  {
    pData = "TRUE";
  }
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%s\n%s%s\n", pComment, pParam, pData);
  ParamString += StringBuffer;

  /* Set SensorOutFile. */
  pComment = "; Output Sensor message to file(TRUE/FALSE)";
  pParam = "SensorOutFile=";
  if (pConfigParam->NmeaOutFile == FALSE)
  {
    pData = "FALSE";
  }
  else
  {
    pData = "TRUE";
  }
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%s\n%s%s\n", pComment, pParam, pData);
  ParamString += StringBuffer;

  /* Set IntervalSec. */
  pComment = "; Positioning interval sec(1-300)";
  pParam = "IntervalSec=";
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%s\n%s%d\n", pComment, pParam, pConfigParam->IntervalSec);
  ParamString += StringBuffer;

  /* Set UartDebugMessage. */
  pComment = "; Uart debug message(NONE/ERROR/WARNING/INFO)";
  pParam = "UartDebugMessage=";
  switch (pConfigParam->UartDebugMessage)
  {
    case PrintError:
      pData = "ERROR";
      break;

    case PrintWarning:
      pData = "WARNING";
      break;

    case PrintInfo:
      pData = "INFO";
      break;

    case PrintNone:
    default:
      pData = "NONE";
      break;
  }
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%s\n%s%s\n", pComment, pParam, pData);
  ParamString += StringBuffer;

  /* End of file. */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "; EOF");
  ParamString += StringBuffer;

  return ParamString;
}


/**
 * @brief Setup positioning.
 * 
 * @return 0 if success, 1 if failure
 */
static void SetupPositioning(void)
{
  int FileCount;

  /* Set default Parameter. */
  Parameter.SatelliteSystem  = eSatGpsGlonassQz1c;
  Parameter.NmeaOutUart      = false;
  Parameter.NmeaOutFile      = true;
  Parameter.SensorOutUart    = false;
  Parameter.SensorOutFile    = true;
  Parameter.IntervalSec      = DEFAULT_INTERVAL_SEC;
  Parameter.UartDebugMessage = PrintNone;

  /* Mount SD card. */
  if(BeginSDCard() != true)
  {
    state = eStateError;
    Led_isState();
  }
  /* make tracker.ini */
  SetupParameter();

  /* Set Gnss debug mode. */
  Gnss.setDebugMode(Parameter.UartDebugMessage);
  if (Gnss.begin(Serial) != 0)
  {
    state = eStateError;
    Led_isState();
  }
  else
  {
    APP_PRINT_I("Gnss begin OK.\n");

    switch (Parameter.SatelliteSystem)
    {
    case eSatGps:
      Gnss.select(GPS);
      break;

    case eSatGpsSbas:
      Gnss.select(GPS);
      Gnss.select(SBAS);
      break;

    case eSatGlonass:
      Gnss.select(GLONASS);
      break;

    case eSatGpsGlonass:
      Gnss.select(GPS);
      Gnss.select(GLONASS);
      break;

    case eSatGpsQz1c:
      Gnss.select(GPS);
      Gnss.select(QZ_L1CA);
      break;

    case eSatGpsQz1cQz1S:
      Gnss.select(GPS);
      Gnss.select(QZ_L1CA);
      Gnss.select(QZ_L1S);
      break;

    case eSatGpsGlonassQz1c:
    default:
      Gnss.select(GPS);
      Gnss.select(GLONASS);
      Gnss.select(QZ_L1CA);
      break;
    }

    Gnss.setInterval(Parameter.IntervalSec);
  }
}

/* ############################################################################################################### */
/* ##            STATE LOOP                                                                                     ## */
/* ############################################################################################################### */

/**
 * @brief Get file number.
 * 
 * @return File count
 */
static unsigned long UpdateFileNumber(void)
{
  int FileCount;
  char IndexData[INDEX_FILE_SIZE];
  int ReadSize = 0;
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
    /* Create a file name to store NMEA(Sensor) data. */
    snprintf(FileNmeaTxt, sizeof(FileNmeaTxt), "Nmea%08d.csv", FileCount);
  }
  else
  {
    /* do nothing. */
  }

  if (Parameter.SensorOutFile == true)
  {
    /* Create a file name to store NMEA(Sensor) data. */
    snprintf(FileSensorTxt, sizeof(FileSensorTxt), "Sensor%08d.csv", FileCount);
  }
  else
  {
    /* do nothing. */
  }

  return FileCount;
}

static void GpsProcessing(void)
{
  int diff = 0;
  static char *pNmeaBuff     = NULL;
  NmeaString = "";

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
  static char *pSensorBuff   = NULL;
  String SensorString = "";
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
  String Sensor;
  char StringBuffer[STRING_BUFFER_SIZE];
  byte rc;/* flag */
  float acc[3];/* acceleration */
  float barom = 0, temp = 0;/* barometer & temperature */

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
  Sensor = "$MOTION,";/* sign name */
  Sensor += "0x0002,";/* device no */

  RtcTime now = RTC.getTime();

  /* Time when rtc was modified by gps. */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%03d,", now.hour(), now.minute(), now.second(), now.nsec() / 1000000);
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

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%f",barom);/* barometer */
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
  /* Check state. */
  time_current = millis();

  time_interval_alive = time_current - time_past_alive;
  if(time_interval_alive >= 1000)
  {
    time_past_alive = time_current;
    Led_isActive();
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

    case  eStateGnss:  /**< Loop is activated */
      Led_isState();
      if(state != state_last)
      {
        Wire.end();
        Gnss.start(HOT_START);
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
