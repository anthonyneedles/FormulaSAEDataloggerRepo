/******************************************************************************
*   DigitalOutput.c
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
*   Comments up to date as of: 04/27/2019
*
*   Created on: 04/26/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "FreeRTOS.h"
#include "task.h"
#include "DigitalOutput.h"

/******************************************************************************
*   PRIVATE DEFINITIONS
******************************************************************************/
#define ENABLE                      0x01U
#define DISABLE                     0x00U
#define ALT_1_GPIO                  0x01U
#define OUTPUT                      0x01U
#define BIT_0_MASK                  0x01U
#define NUM_OUTPUTS                 0x08U

/******************************************************************************
*   PRIVATE ENUMERATIONS
******************************************************************************/
typedef enum
{
    ON, OFF
} OutState_t;

typedef enum
{
    FIVE_VOLTS, TWELVE_VOLTS
} OutPower_t;


typedef enum
{
    DOUT1, DOUT2, DOUT3, DOUT4, DOUT5, DOUT6, DOUT7, DOUT8
} OutName_t;

/******************************************************************************
*   PRIVATE STRUCTURES
******************************************************************************/
typedef struct DigitalOut_t
{
    OutName_t name;
    uint8_t portA_pin_num;
    OutState_t state;
    OutPower_t power;
} DigitalOut_t;

/******************************************************************************
*   PRIVATE FUNCTION PROTOTYPES
******************************************************************************/
void digitalOutputChange(DigitalOut_t);

/******************************************************************************
*   PRIVATE VARIABLES
******************************************************************************/
static DigitalOut_t doutConfigs[NUM_OUTPUTS] =
{
    { DOUT1, 17U, OFF, FIVE_VOLTS },
    { DOUT2, 16U, OFF, FIVE_VOLTS },
    { DOUT3, 15U, OFF, FIVE_VOLTS },
    { DOUT4, 14U, OFF, FIVE_VOLTS },
    { DOUT5, 13U, OFF, FIVE_VOLTS },
    { DOUT6, 12U, OFF, FIVE_VOLTS },
    { DOUT7, 11U, OFF, FIVE_VOLTS },
    { DOUT8, 10U, OFF, FIVE_VOLTS }
};

/******************************************************************************
*   DigitalOutputInit() - Public function to initialize all digital outputs in
*   OFF state and supplying 5V.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void DigitalOutputInit()
{
    uint8_t pta_num;

    /* Enable PORTA (pta) clock (all DOUT pins are in PORTA) */
    SIM->SCGC5 |= SIM_SCGC5_PORTA(ENABLE);

    /* Configure all DOUT pin MUXs to ALT 1 (GPIO), and configure all GPIO as
     * outputs. Sets all DOUT to initial state/power, which is OFF/5V */
    for(uint8_t dcount = 0; dcount < NUM_OUTPUTS; dcount++)
    {
        pta_num = doutConfigs[dcount].portA_pin_num;

        PORTA->PCR[pta_num] = ((PORTA->PCR[pta_num] & ~PORT_PCR_MUX_MASK) |
                                PORT_PCR_MUX(ALT_1_GPIO));

        GPIOA->PDDR |= GPIO_PDDR_PDD(OUTPUT << pta_num);

        digitalOutputChange(doutConfigs[dcount]);
    }
}

/******************************************************************************
*   digitalOutPutChange() - Private function to modify private digital output
*   configuration structures. Sets/clears GPIO pins to provide requested state
*   and power output.
*
*   Parameters:
*
*       DigitalOut_t dout - Digital output structure to be changed.
*
*   Return: None
******************************************************************************/
void digitalOutputChange(DigitalOut_t dout)
{
//    switch(dout.state)
//    {
//        case ON:
//            GPIOA->PDOR |= GPIO_PDOR_PDO(ENABLE << dout.portA_pin_num);
//            break;
//
//        case OFF:
//            GPIOA->PDOR &= ~GPIO_PDOR_PDO(ENABLE << dout.portA_pin_num);
//            break;
//
//        default:
//            break;
//    }

    switch(dout.power)
    {
        case FIVE_VOLTS:
            GPIOA->PDOR &= ~GPIO_PDOR_PDO(ENABLE << dout.portA_pin_num);
            break;

        case TWELVE_VOLTS:
            GPIOA->PDOR |= GPIO_PDOR_PDO(ENABLE << dout.portA_pin_num);
            break;

        default:
            break;
    }
}
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
void DigitalOutputSet(DigitalOutMsg_t msg)
{
    uint8_t state_bit;
    uint8_t power_bit;

    /* Convert state/power message to individual state/power bits for each
     * DOUT (i.e. msg.state_field bit 0 corresponds to DOUT1's desired state,
     * msg.state_field bit 1 corresponds to DOUT2's state, etc.)
     * A power bit of 1 corresponds to a power of 12V */
    for(uint8_t dcount = 0; dcount < NUM_OUTPUTS; dcount++)
    {
        state_bit = ((msg.state_field >> dcount) & (uint8_t)BIT_0_MASK);
        power_bit = ((msg.power_field >> dcount) & (uint8_t)BIT_0_MASK);

        doutConfigs[dcount].state = (state_bit ? ON : OFF);
        doutConfigs[dcount].power = (power_bit ? TWELVE_VOLTS : FIVE_VOLTS);

        digitalOutputChange(doutConfigs[dcount]);
    }
}
