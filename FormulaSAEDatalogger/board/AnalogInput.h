/******************************************************************************
*   AnalogInput.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Analog Input module of the Formula SAE Datalogger provides public
*   functions to set the power supply (5V/12V) provided to the 4 analog
*   sensors as well as the required input conditioning MUX selection. The
*   supplies are rated to supply a maximum of 20mA a either 5V or 12V per
*   sensor. The analog signals will be measured via ADC1 channels, with PIT1
*   providing hardware triggering at 8kHz.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/04/2019
*
*   Created on: 04/29/2019
*   Author: Anthony Needles
******************************************************************************/
#ifndef ANALOGINPUT_H_
#define ANALOGINPUT_H_

typedef struct AnlgInMsg_t
{
    uint8_t power_state_field;
    uint32_t sampling_rate_field;
} AnlgInMsg_t;

/******************************************************************************
*   AnlgInInit() - Public function to configure all GPIO used as power and
*   conditioning selects as outputs. Initializes Analog In data structure and
*   creates Mutex to protect data structure. Also initializes PIT1 to trigger
*   ADC1 at 8kHz. Configures and calibrates ADC1.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void AnlgInInit(void);

/******************************************************************************
*   AnlgInSet() - Public function to set power and conditioning pins to achieve
*   requested results from received user message. Saves the new Analog In data
*   from message in Mutex protected private data structure.
*
*   Parameters:
*
*       AnlgInMsg_t msg - Message structure received from telemetry unit
*       with 8 bit power and state field (msg.power_state_field) and 32 bit
*       sampling rates field (msg.sampling_rate_field).
*
*       msg.power_state_field[0:3] corresponds to AIN[1:4] power, with a bit of
*       0 representing a power output of 5V (1 = 12V).
*
*       msg.power_state_field[4:7] corresponds to AIN[1:4] state, with a bit of
*       0 representing a non-active sensor input (1 = active).
*
*       msg.sampling_rate_field[0:7] corresponds to AIN1 sampling rate,
*       msg.sampling_rate_field[8:15] corresponds to AIN2 sampling rate, etc.
*
*   Return: None
******************************************************************************/
void AnlgInSet(AnlgInMsg_t);

/******************************************************************************
*   anlginCalibrateADC1() - Private function that calibrates ADC1 after reset
*   and before any conversion are initiated.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void PIT1_IRQHandler(void);

#endif /* ANALOGINPUT_H_ */
