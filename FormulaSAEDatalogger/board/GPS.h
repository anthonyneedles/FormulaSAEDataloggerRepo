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
*   Communication from the GPS unit occurs on UART4 at 1Hz.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/12/2019
*
*   Created on: 05/10/2019
*   Author: Anthony Needles
******************************************************************************/
#ifndef GPS_H_
#define GPS_H_

/******************************************************************************
*   GPSUART4Init() - Public function to configure UART4 for reception of GPS
*   NMEA time and date data. This data will serve as the starting point of
*   the datalogger's timestamps.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void GPSUART4Init(void);

//void UART4_RX_TX_IRQHandler(void);

#endif /* GPS_H_ */
