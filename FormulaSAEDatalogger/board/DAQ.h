/*
 * DAQ.h
 *
 *  Created on: Jun 3, 2019
 *      Author: aneed
 */

#ifndef DAQ_H_
#define DAQ_H_

#define DAQ1                    0U
#define DAQ2                    1U
#define DAQ3                    2U
#define DAQ4                    3U
#define DAQ5                    4U
#define DAQ6                    5U
#define DAQ7                    6U
#define DAQ8                    7U

#define SENSOR1                 0U
#define SENSOR2                 1U
#define SENSOR3                 2U
#define SENSOR4                 3U

typedef struct sensor_t
{
    uint16_t data;
    uint8_t power;
    uint8_t type;
    uint8_t time_sec;
    uint8_t time_cs;
} sensor_t;

typedef struct daq_data_t
{
    sensor_t sensor[4];
} daq_data_t;

typedef struct daq_msg_t
{
    uint8_t power_fields[8];
    uint8_t type_fields[8];
} daq_msg_t;

void DAQInit(void);

void DAQSet(daq_msg_t);

void DAQGetData(daq_data_t *);

#endif /* DAQ_H_ */
