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
*   DigitalOutput_GENERIC.c/.h supports dynamically changing DOUT ports/pins,
*   bit since the DOUT ports/pins are set, this module is used as it is much
*   faster and smaller.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/22/2019
*
*   Created on: 04/26/2019
*   Author: Anthony Needles
******************************************************************************/
#ifndef DIGITALOUTPUT_H_
#define DIGITALOUTPUT_H_

/******************************************************************************
*   Public Definitions
******************************************************************************/
typedef struct dout_msg_t
{
    uint8_t state_field;
    uint8_t power_field;
} dout_msg_t;

/******************************************************************************
*   Public Function Prototypes
******************************************************************************/
/******************************************************************************
*   DOutInit() - Public function to initialize all digital outputs in OFF
*   state and supplying 5V. All configuration settings are stored in
*   doutConfigs structure.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void DOutInit(void);

/******************************************************************************
*   DOutSet() - Public function to set digital output states and powers via
*   requested message structure.
*
*   Parameters:
*
*       dout_msg_t msg - Message structure received from telemetry unit with
*       8 bit state field (msg.state_field) and 8 bit power field
*       (msg.power_field). msg.state_field bit 0 corresponds to DOUT1's desired
*       state, msg.state_field bit 1 corresponds to DOUT2's state, etc.
*
*   Return: None
******************************************************************************/
void DOutSet(dout_msg_t);

#endif /* DIGITALOUTPUT_H_ */
