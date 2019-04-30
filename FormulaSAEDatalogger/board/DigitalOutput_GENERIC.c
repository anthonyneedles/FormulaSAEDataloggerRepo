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
*   Comments up to date as of: 04/29/2019
*
*   Created on: 04/26/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "FreeRTOS.h"
#include "task.h"
#include <DigitalOutput_GENERIC.h>

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
typedef struct PinID_t
{
    uint8_t num;
    PORT_Type *port_base;
    GPIO_Type *gpio_base;
} PinID_t;

typedef struct DigitalOut_t
{
    OutName_t name;
    PinID_t power_pin;
    OutPower_t power;
    PinID_t state_pin;
    OutState_t state;
} DigitalOut_t;

/******************************************************************************
*   PRIVATE FUNCTION PROTOTYPES
******************************************************************************/
void digitalOutputChange_GENERIC(DigitalOut_t);

/******************************************************************************
*   PRIVATE VARIABLES
******************************************************************************/
/* This is the main configuration structure for the DOUTs. Be sure necessary
 * PORT clock is enabled in DigitalOutputInit() if pins are changed. */
static DigitalOut_t doutConfigs[NUM_OUTPUTS] =
{
    { DOUT1, { 17U, PORTA, GPIOA }, FIVE_VOLTS, {  1U, PORTB, GPIOB }, OFF },
    { DOUT2, { 16U, PORTA, GPIOA }, FIVE_VOLTS, { 28U, PORTA, GPIOA }, OFF },
    { DOUT3, { 15U, PORTA, GPIOA }, FIVE_VOLTS, {  0U, PORTB, GPIOB }, OFF },
    { DOUT4, { 14U, PORTA, GPIOA }, FIVE_VOLTS, { 29U, PORTA, GPIOA }, OFF },
    { DOUT5, { 13U, PORTA, GPIOA }, FIVE_VOLTS, { 27U, PORTA, GPIOA }, OFF },
    { DOUT6, { 12U, PORTA, GPIOA }, FIVE_VOLTS, { 24U, PORTA, GPIOA }, OFF },
    { DOUT7, { 11U, PORTA, GPIOA }, FIVE_VOLTS, { 26U, PORTA, GPIOA }, OFF },
    { DOUT8, { 10U, PORTA, GPIOA }, FIVE_VOLTS, { 25U, PORTA, GPIOA }, OFF }
};

/******************************************************************************
*   DigitalOutputInit() - Public function to initialize all digital outputs in
*   OFF state and supplying 5V. All configuration settings are stored in
*   doutConfigs structure.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void DigitalOutputInit_GENERIC()
{
    uint8_t pin_num;
    PORT_Type *port_base;
    GPIO_Type *gpio_base;

    /* Enable PORTA and PORTB clocks (all DOUT pins are in PORTA or PORTB). */
    SIM->SCGC5 |= SIM_SCGC5_PORTA(ENABLE);
    SIM->SCGC5 |= SIM_SCGC5_PORTB(ENABLE);

    /* Configure all DOUT pin MUXs to ALT 1 (GPIO), and configure all GPIO as
     * outputs. This is done for both the power and state pins for each DOUT,
     * with port name/gpio name/pin number defined in DOUT config structure.
     * Sets all DOUT pins to initial state/power, which is OFF/5V, as specified
     * in DOUT config structure. */
    for(uint8_t dcount = 0; dcount < NUM_OUTPUTS; dcount++)
    {
        /* Power pin configuration */
        pin_num = doutConfigs[dcount].power_pin.num;
        port_base = doutConfigs[dcount].power_pin.port_base;
        gpio_base = doutConfigs[dcount].power_pin.gpio_base;

        port_base->PCR[pin_num] = ((port_base->PCR[pin_num] & ~PORT_PCR_MUX_MASK) |
                                   PORT_PCR_MUX(ALT_1_GPIO));

        gpio_base->PDDR |= GPIO_PDDR_PDD(OUTPUT << pin_num);

        /* State pin configuration */
        pin_num = doutConfigs[dcount].state_pin.num;
        port_base = doutConfigs[dcount].state_pin.port_base;

        port_base->PCR[pin_num] = ((port_base->PCR[pin_num] & ~PORT_PCR_MUX_MASK) |
                                   PORT_PCR_MUX(ALT_1_GPIO));

        gpio_base->PDDR |= GPIO_PDDR_PDD(OUTPUT << pin_num);

        /* Set configurations */
        digitalOutputChange_GENERIC(doutConfigs[dcount]);
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
void digitalOutputChange_GENERIC(DigitalOut_t dout)
{
    uint8_t pin_num;
    GPIO_Type *gpio_base;

    /* Write to output state pin */
    pin_num = dout.state_pin.num;
    gpio_base = dout.state_pin.gpio_base;

    switch(dout.state)
    {
        case ON:
            gpio_base->PDOR |= GPIO_PDOR_PDO(ENABLE << pin_num);
            break;

        case OFF:
            gpio_base->PDOR &= ~GPIO_PDOR_PDO(ENABLE << pin_num);
            break;

        default:
            break;
    }

    /* Write to output power pin */
    pin_num = dout.power_pin.num;
    gpio_base = dout.power_pin.gpio_base;

    switch(dout.power)
    {
        case FIVE_VOLTS:
            gpio_base->PDOR &= ~GPIO_PDOR_PDO(ENABLE << pin_num);
            break;

        case TWELVE_VOLTS:
            gpio_base->PDOR |= GPIO_PDOR_PDO(ENABLE << pin_num);
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
void DigitalOutputSet_GENERIC(DigitalOutMsg_t msg)
{
    uint8_t state_bit;
    uint8_t power_bit;

    /* Convert state/power message to individual state/power bits for each
     * DOUT (i.e. msg.state_field bit 0 corresponds to DOUT1's desired state,
     * msg.state_field bit 1 corresponds to DOUT2's state, etc.)
     * A power bit of 1 corresponds to a power of 12V and a state bit of 1
     * corresponds to an ON DOUT. */
    for(uint8_t dcount = 0; dcount < NUM_OUTPUTS; dcount++)
    {
        state_bit = ((msg.state_field >> dcount) & (uint8_t)BIT_0_MASK);
        power_bit = ((msg.power_field >> dcount) & (uint8_t)BIT_0_MASK);

        doutConfigs[dcount].state = (state_bit ? ON : OFF);
        doutConfigs[dcount].power = (power_bit ? TWELVE_VOLTS : FIVE_VOLTS);

        digitalOutputChange_GENERIC(doutConfigs[dcount]);
    }
}

