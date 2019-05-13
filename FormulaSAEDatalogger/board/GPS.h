/*
 * GPS.h
 *
 *  Created on: Mar 18, 2019
 *      Author: aneed
 */

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
