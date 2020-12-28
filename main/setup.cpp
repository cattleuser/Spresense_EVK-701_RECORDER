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
 * @brief global APIs
 */
extern void SetupPositioning(void);
extern void Led_isState();

/**
 * @brief private APIs
 */
static int ReadParameter(ConfigParam *pConfigParam);
static void WriteParameter(ConfigParam *pConfigParam);
static String MakeParameterString(ConfigParam *pConfigParam);
static int ParamCompare(const char *Input , const char *Refer);
static int SetupParameter(void);

/**
 * @brief global variables and functions
 */
extern ConfigParam Parameter;  /**< Configuration parameters */
extern word state;
extern SpGnss Gnss;                 /**< SpGnss object */
extern unsigned long write_size;

/**
 * @brief Setup positioning.
 * 
 * @return 0 if success, 1 if failure
 */
void Led_isState()
{
  if(LED_DEBUG_MODE)
  {
    switch(state)
    {
      case  eStateIdle:
        ledOn(PIN_LED1); ledOff(PIN_LED2); ledOff(PIN_LED3);
        break;
  
      case eStateRenewFile:
        ledOn(PIN_LED1); ledOn(PIN_LED2); ledOff(PIN_LED3);
        break;
  
      case eStateGnssNonFix:
        ledOff(PIN_LED1); ledOff(PIN_LED2); ledOn(PIN_LED3);
        break;
  
      case eStateSensor:
        ledOn(PIN_LED1); ledOff(PIN_LED2); ledOn(PIN_LED3);
        break;
  
      case eStateError:
        ledOff(PIN_LED1); ledOn(PIN_LED2); ledOn(PIN_LED3);
        /* module stop. */
        while(1);
        break;
  
      case eStateWriteError:
        ledOn(PIN_LED1); ledOn(PIN_LED2); ledOn(PIN_LED3);
        /* module stop. */
        while(1);
        break;
  
      default:
        ledOn(PIN_LED1); ledOn(PIN_LED2); ledOn(PIN_LED3);
        /* module stop. */
        while(1);
        break;
    }
  }
  else
  {
    switch(state)
    {
      case eStateGnssNonFix:
        ledOff(PIN_LED1); ledOn(PIN_LED2); ledOff(PIN_LED3);
        break;

      case eStateError:
        ledOff(PIN_LED1); ledOff(PIN_LED2); ledOn(PIN_LED3);
        /* module stop. */
        while(1);
        break;
  
      case eStateWriteError:
        ledOff(PIN_LED1); ledOff(PIN_LED2); ledOn(PIN_LED3);
        /* module stop. */
        while(1);
        break;

      default:
        ledOff(PIN_LED1); ledOff(PIN_LED2); ledOff(PIN_LED3);
        break;
    }
  }/* LED_DEBUG_MODE */
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
 * @brief Create an ini file based on the current parameters.
 * 
 * @param [in] pConfigParam Configuration parameters
 * @return 0 if success, -1 if failure
 */
void WriteParameter(ConfigParam *pConfigParam)
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
 * @brief Setup configuration parameters.
 * 
 * @return 0 if success, -1 if failure
 */
int SetupParameter(void)
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
 * @brief Compare parameter.
 * 
 * @param [in] Input Parameter to compare
 * @param [in] Refer Reference parameter
 * @return 0 if equal
 */
int ParamCompare(const char *Input , const char *Refer)
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

extern void SetupPositioning(void)
{
  /* Set default Parameter. */
  Parameter.SatelliteSystem  = SATELLIT_ESYSTEM;
  Parameter.NmeaOutUart      = NMEA_OUT_UART;
  Parameter.NmeaOutFile      = NMEA_OUT_FILE;
  Parameter.SensorOutUart    = SENSOR_OUT_UART;
  Parameter.SensorOutFile    = SENSOR_OUT_FILE;
  Parameter.IntervalSec      = INTERVAL_SEC;
  Parameter.UartDebugMessage = UART_DEBUG_MESSAGE;

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
