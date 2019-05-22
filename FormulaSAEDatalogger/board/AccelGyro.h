/******************************************************************************
*   AccelGyro.h
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Accel/Gyro module of the Formula SAE Datalogger initializes and
*   establishes an I2C communication link with the on-board accelerometer and
*   gyroscope unit. The relevant data is then read from the unit and made
*   available other peripherals.
*
*   Accelerometer/Gyroscope Unit: LSM6DS3
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/22/2019
*
*   Created on: 05/15/2019
*   Author: Anthony Needles
******************************************************************************/
#ifndef ACCELGYRO_H_
#define ACCELGYRO_H_

/******************************************************************************
*   Public Definitions
******************************************************************************/
/* Public message structure to set accel/gyro sampling configurations. */
typedef struct ag_msg_t
{
    uint8_t sampling_rate_field;
} ag_msg_t;

/* Structure definition for a sensor that collects data in 3 dimensions. */
typedef struct sensor_3d_t
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} sensor_3d_t;

/* Structure definition for Accel/Gyro data and configurations. */
typedef struct ag_data_t
{
    uint8_t sampling_rate;
    sensor_3d_t accel_data;
    sensor_3d_t gyro_data;
} ag_data_t;

/******************************************************************************
*   Public Function Prototypes
******************************************************************************/
/******************************************************************************
*   AGInit() - Public function to configure I2C3 for reception of accelerometer
*   and gyroscope data. Configuration of Accel/Gyro unit requires setting of
*   output data rate, data range, anti-aliasing filter cutoff, etc. in the
*   internal Accel/Gyro registers.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void AGInit(void);

/******************************************************************************
*   I2C3_IRQHandler() - Interrupt handler for I2C3 IICIF. This triggers upon
*   byte transfer completion (including ACK/NACK). Clears IICIF flag and posts
*   task notification to Accel/Gyro Sampler Task. This allows non-blocking I2C
*   driver operation.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void I2C3_IRQHandler(void);

/******************************************************************************
*   PIT2_IRQHandler() - Interrupt handler for PIT2 TIF. Will enter every 10ms
*   (100Hz) to trigger a new accel/gyro data read via I2C3 driver.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void PIT2_IRQHandler(void);

/******************************************************************************
*   AGSet() - Public function to set accel/gyro sampling rate and sampling
*   on/off state to achieve requested results from received user message. Saves
*   the new configuration data from the message in Mutex protected private data
*   structure.
*
*   Parameters:
*
*       AnlgInMsg_t msg - Message structure received from telemetry unit
*       with 8 bit power and state field (msg.power_state_field) and 32 bit
*       sampling rates field (msg.sampling_rate_field).
*
*       msg.sampling_rate_field[2:0] corresponds to the requested sample rate
*       of the accel/gyro. Valid values are:
*
*           0x0 - SR of 0Hz,   BW of 0Hz
*           0x1 - SR of 25Hz,  BW of 12.5Hz
*           0x2 - SR of 52Hz,  BW of 26Hz
*           0x3 - SR of 104Hz, BW of 52Hz
*           0x4 - SR of 208Hz, BW of 104Hz
*
*   Return: None
******************************************************************************/
void AGSet(ag_msg_t);

/******************************************************************************
*   AGGetData() - Public function to copy current accel/gyro data structure
*   for use in transmission/storage.
*
*   Parameters:
*
*       ag_data_t *ldata - Pointer to caller-side data structure which will
*       have current data copied to it.
*
*   Return: None
******************************************************************************/
void AGGetData(ag_data_t *);

#endif /* ACCELGYRO_H_ */
