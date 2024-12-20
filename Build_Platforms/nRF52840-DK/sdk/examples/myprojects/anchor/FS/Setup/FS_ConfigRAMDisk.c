/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : FS_ConfigRAMDisk.c
Purpose : Configuration file for FS with RAM disk
*/

/*********************************************************************
*
*       #include section
*
**********************************************************************
*/
#include "FS.h"

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/
#ifndef   ALLOC_SIZE
  #define ALLOC_SIZE          0x800     // Memory pool for the file system in bytes.
#endif

#ifndef   NUM_LOG_SECTORS
  #define NUM_LOG_SECTORS     16        // Number of logical sectors on the storage.
#endif

#ifndef   LOG_SECTOR_SIZE
  #define LOG_SECTOR_SIZE     512       // Size of a logical sector in bytes.
#endif

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
static U32 _aMemBlock[ALLOC_SIZE / 4];                          // Memory pool used for semi-dynamic allocation.
static U32 _aRAMDisk[(NUM_LOG_SECTORS * LOG_SECTOR_SIZE) / 4];  // Memory to be used as storage.

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

/*********************************************************************
*
*       FS_X_AddDevices
*
*  Function description
*    Configures the file system.
*
*  Additional information
*    This function is responsible for configuring the file system at
*    runtime and it has to be implemented by any application that
*    uses emFile.
*
*    FS_X_AddDevices() is called during the file system initialization
*    from either FS_Init() or FS_STORAGE_Init(). At the minimum, this
*    function has to provide memory for the file system to work with
*    either via FS_AssignMemory() or FS_SetMemHandler(). In addition,
*    FS_X_AddDevices() has to add at least one device driver to the
*    file system via FS_AddDevice() and to configure the device driver.
*
*    API functions that access a storage device may not be called in
*    FS_X_AddDevices() because the device drivers may not be ready
*    at this point to perform any data transfers.
*/
void FS_X_AddDevices(void) {
  //
  // Give the file system memory to work with.
  //
  FS_AssignMemory(&_aMemBlock[0], sizeof(_aMemBlock));
  //
  // Add and configure the RAMDISK driver.
  //
  FS_AddDevice(&FS_RAMDISK_Driver);
  FS_RAMDISK_Configure(0, _aRAMDisk, LOG_SECTOR_SIZE, NUM_LOG_SECTORS);
#if FS_SUPPORT_FILE_BUFFER
  //
  // Enable the file buffer to increase the performance when reading/writing a small number of bytes.
  //
  FS_ConfigFileBufferDefault(LOG_SECTOR_SIZE, FS_FILE_BUFFER_WRITE);
#endif // FS_SUPPORT_FILE_BUFFER
}

/*********************************************************************
*
*       FS_X_GetTimeDate
*
*  Function description
*    Returns the current time and date.
*
*  Return value
*    Current time and date in a format suitable for the file system.
*
*  Additional information
*    This function is called by the file system in order to generate
*    the time stamps for the accessed files and directories.
*    FS_X_GetTimeDate() is not required when the application accesses
*    the storage device only via the Storage layer and not via the
*    File system layer.
*
*    Alternatively, the application can register a runtime a callback
*    function via FS_SetTimeDateCallback() that returns the date and time.
*
*    The return value is formatted as follows:
*    +-----------+----------------------------------+
*    | Bit range | Description                      |
*    +-----------+----------------------------------+
*    | 0-4       | 2-second count (0-29)            |
*    +-----------+----------------------------------+
*    | 5-10      | Minutes (0-59)                   |
*    +-----------+----------------------------------+
*    | 11-15     | Hours (0-23)                     |
*    +-----------+----------------------------------+
*    | 16-20     | Day of month (1-31)              |
*    +-----------+----------------------------------+
*    | 21-24     | Month of year (1-12)             |
*    +-----------+----------------------------------+
*    | 25-31     | Count of years from 1980 (0-127) |
*    +-----------+----------------------------------+
*/
U32 FS_X_GetTimeDate(void) {
  U32 r;
  U16 Sec, Min, Hour;
  U16 Day, Month, Year;

  Sec   = 0;        // 0 based.  Valid range: 0..59
  Min   = 0;        // 0 based.  Valid range: 0..59
  Hour  = 0;        // 0 based.  Valid range: 0..23
  Day   = 1;        // 1 based.    Means that 1 is 1. Valid range is 1..31 (depending on month)
  Month = 1;        // 1 based.    Means that January is 1. Valid range is 1..12.
  Year  = 0;        // 1980 based. Means that 2007 would be 27.
  r   = Sec / 2 + (Min << 5) + (Hour  << 11);
  r  |= (U32)(Day + (Month << 5) + (Year  << 9)) << 16;
  return r;
}

/*************************** End of file ****************************/
