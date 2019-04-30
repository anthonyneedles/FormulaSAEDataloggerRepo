/******************************************************************************
*   DigitalOutput.h
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Digital Output module of the Formula SAE Datalogger provides public
*   functions to set the state (on/off and 5V/12V power) of the 8 open drain
*   digital outputs provided to off board devices. These open drain outputs
*   are rated to supply a maximum of 20mA at either 5V or 12V. An example
*   application of a digital output is to turn a dashboard service light on
*   and off.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 04/29/2019
*
*   Created on: 04/26/2019
*   Author: Anthony Needles
******************************************************************************/
#ifndef DIGITALOUTPUT_GENERIC_H_
#define DIGITALOUTPUT_GENERIC_H_

/******************************************************************************
*   PUBLIC STRUCTURES
******************************************************************************/
typedef struct DigitalOutMsg_t
{
    uint8_t state_field;
    uint8_t power_field;
} DigitalOutMsg_t;

/******************************************************************************
*   PUBLIC FUNCTION PROTOTYPES
******************************************************************************/
/******************************************************************************
*   DigitalOutputInit() - Public function to initialize all digital outputs in
*   OFF state and supplying 5V. All configuration settings are stored in
*   doutConfigs structure.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void DigitalOutputInit_GENERIC(void);

/******************************************************************************
*   DigitalOutputSet() - Public function to set digital output states and
*   powers via requested message structure.
*
*   Parameters:
*
*       DigitalOutMsg_t msg - Message structure received from telemetry unit.
*
*   Return: None
******************************************************************************/
void DigitalOutputSet_GENERIC(DigitalOutMsg_t);

#endif /* DIGITALOUTPUT_GENERIC_H_ */
