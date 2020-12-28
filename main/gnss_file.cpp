/*
 *  gnss_file.cpp - Handling I/O operation on the SD card
 *  Copyright 2017 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/**
 * @file gnss_file.cpp
 * @author Sony Semiconductor Solutions Corporation
 * @brief Handling I/O operation on the SD card
 */

#include "gnss_file.h"

SDClass theSD;  /**< SDClass object */

boolean BeginSDCard(void)
{
  if (!theSD.begin()) {
    return false;
  }
  return true;
}

volatile int WriteBinary(const char* pBuff, const char* pName, unsigned long write_size, int flag)
{
  unsigned long write_result = 0;
  File myFile;

  if (theSD.exists("/") == false) {
    return 0;
  }

  if (write_size != 0)
  {
    /* Open file. */
    myFile = theSD.open(pName, flag);

    if (myFile == NULL)
    {
      /* if the file didn't open, print an error. */
    }
    else
    {
      /* Write file. */
      write_result = myFile.write(pBuff, write_size);
      if (write_result != write_size)
      {
        /* Write error. */
      }
      else
      {
        /* Write OK. */
      }

      /* Close file. */
      myFile.close();
    }
  }

  return write_result;
}

int WriteChar(const char* pBuff, const char* pName, int flag)
{
  unsigned long write_size = strlen(pBuff);

  return WriteBinary(pBuff, pName, write_size, flag);
}

int ReadChar(char* pBuff, int BufferSize, const char* pName, int flag)
{
  int read_result = 0;
  File myFile;

  /* Open file. */
  if (theSD.exists(pName) == false) {
    return 0;
  }
  myFile = theSD.open(pName, flag);
  if (myFile == NULL)
  {
    /* if the file didn't open, print an error. */
  }
  else
  {
    /* Write file. */
    read_result = myFile.read(pBuff, BufferSize);
    if (read_result == 0)
    {
      /* Read error. */
    }
    else
    {
      /* Read OK. */
    }

    /* Close file. */
    myFile.close();
  }

  return read_result;
}

int Remove(const char* pName)
{
  return theSD.remove(pName);
}

boolean IsFileExist(const char* pName)
{
  return theSD.exists(pName);
}
