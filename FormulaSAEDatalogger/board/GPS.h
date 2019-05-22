/******************************************************************************
*   GPS.h
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The GPS module of the Formula SAE Datalogger obtains real-world time and
*   date data for accurate timestamps. This configuration will be done once upon
*   program starting initialization to set the time/date starting point.
*   Communication from the GPS unit occurs on UART4 at 9600 baud.
*
*   GPS Unit: L80-R
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/22/2019
*
*   Created on: 05/10/2019
*   Author: Anthony Needles
******************************************************************************/
#ifndef GPS_H_
#define GPS_H_

/******************************************************************************
*   Public Definitions
******************************************************************************/
/* Time and date structures held as decimal values (yymmdd hhmmss.uu) */
typedef struct gps_data_t
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t ms;
} gps_data_t;

/******************************************************************************
*   Public Function Prototypes
******************************************************************************/
/******************************************************************************
*   GPSInit() - Public function to configure UART4 for reception of GPS
*   NMEA time and date data. This data will serve as the starting point of
*   the datalogger's timestamps.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void GPSInit(void);

/******************************************************************************
*   PIT0_IRQHandler() - Interrupt handler for PIT0 TIF (timer interrupt) flag.
*   Occurs at 100Hz. Triggers 10ms increment by posting task notify to
*   gpsTenMSTask.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void PIT0_IRQHandler(void);

/******************************************************************************
*   GPSGetData() - Public function to copy current GPS data structure
*   for use in transmission/storage.
*
*   Parameters:
*
*       gps_time_date_data_t *ldata - Pointer to caller-side data structure
*       which will have current data copied to it.
*
*   Return: None
******************************************************************************/
void GPSGetData(gps_data_t *);

#endif /* GPS_H_ */
