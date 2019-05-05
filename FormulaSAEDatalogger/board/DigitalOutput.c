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
*   DigitalOutput_GENERIC.c/.h supports dynamically changing DOUT ports/pins,
*   bit since the DOUT ports/pins are set, this module is used as it is much
*   faster and smaller.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/04/2019
*
*   Created on: 04/26/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "DigitalOutput.h"

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                      0x01U
#define ALT_1_GPIO                  0x01U
#define OUTPUT                      0x01U
#define BIT_0_MASK        ((uint8_t)0x01U)

/* DOUT pin numbers, DOUT1 and DOUT3 are PORTB, all else are PORTA */
#define DOUT1_STATE_PIN_NUM            1U
#define DOUT2_STATE_PIN_NUM           28U
#define DOUT3_STATE_PIN_NUM            0U
#define DOUT4_STATE_PIN_NUM           29U
#define DOUT5_STATE_PIN_NUM           27U
#define DOUT6_STATE_PIN_NUM           24U
#define DOUT7_STATE_PIN_NUM           26U
#define DOUT8_STATE_PIN_NUM           25U

/* DOUT pin numbers, all are PORTA */
#define DOUT1_POWER_PIN_NUM           17U
#define DOUT2_POWER_PIN_NUM           16U
#define DOUT3_POWER_PIN_NUM           15U
#define DOUT4_POWER_PIN_NUM           14U
#define DOUT5_POWER_PIN_NUM           13U
#define DOUT6_POWER_PIN_NUM           12U
#define DOUT7_POWER_PIN_NUM           11U
#define DOUT8_POWER_PIN_NUM           10U

/* Bit # corresponding to certain DOUT in 8-bit state/power field message */
#define DOUT1_BIT_NUM        ((uint8_t)0U)
#define DOUT2_BIT_NUM        ((uint8_t)1U)
#define DOUT3_BIT_NUM        ((uint8_t)2U)
#define DOUT4_BIT_NUM        ((uint8_t)3U)
#define DOUT5_BIT_NUM        ((uint8_t)4U)
#define DOUT6_BIT_NUM        ((uint8_t)5U)
#define DOUT7_BIT_NUM        ((uint8_t)6U)
#define DOUT8_BIT_NUM        ((uint8_t)7U)

/******************************************************************************
*   Private Macros
******************************************************************************/
/* Macros accepting OFF/ON (0/1) to either set or clear DOUT state pin*/
#define DOUT1_STATE_SET(x) (GPIOB->PDOR |= ((x) << DOUT1_STATE_PIN_NUM))
#define DOUT2_STATE_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_STATE_PIN_NUM))
#define DOUT3_STATE_SET(x) (GPIOB->PDOR |= ((x) << DOUT1_STATE_PIN_NUM))
#define DOUT4_STATE_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_STATE_PIN_NUM))
#define DOUT5_STATE_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_STATE_PIN_NUM))
#define DOUT6_STATE_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_STATE_PIN_NUM))
#define DOUT7_STATE_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_STATE_PIN_NUM))
#define DOUT8_STATE_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_STATE_PIN_NUM))

/* Macros accepting FIVE_VOLTS/TWELVE_VOLTS (0/1) to either set or clear DOUT
 * power pin*/
#define DOUT1_POWER_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_POWER_PIN_NUM))
#define DOUT2_POWER_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_POWER_PIN_NUM))
#define DOUT3_POWER_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_POWER_PIN_NUM))
#define DOUT4_POWER_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_POWER_PIN_NUM))
#define DOUT5_POWER_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_POWER_PIN_NUM))
#define DOUT6_POWER_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_POWER_PIN_NUM))
#define DOUT7_POWER_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_POWER_PIN_NUM))
#define DOUT8_POWER_SET(x) (GPIOA->PDOR |= ((x) << DOUT1_POWER_PIN_NUM))

/* Macros accepting 8 bit field to find and isolate DOUT's relevant bit and
 * shift over to bit 0 position for relevant SET function */
#define DOUT1_BIT(x) (((x) >> DOUT1_BIT_NUM) & BIT_0_MASK)
#define DOUT2_BIT(x) (((x) >> DOUT2_BIT_NUM) & BIT_0_MASK)
#define DOUT3_BIT(x) (((x) >> DOUT3_BIT_NUM) & BIT_0_MASK)
#define DOUT4_BIT(x) (((x) >> DOUT4_BIT_NUM) & BIT_0_MASK)
#define DOUT5_BIT(x) (((x) >> DOUT5_BIT_NUM) & BIT_0_MASK)
#define DOUT6_BIT(x) (((x) >> DOUT6_BIT_NUM) & BIT_0_MASK)
#define DOUT7_BIT(x) (((x) >> DOUT7_BIT_NUM) & BIT_0_MASK)
#define DOUT8_BIT(x) (((x) >> DOUT8_BIT_NUM) & BIT_0_MASK)

/******************************************************************************
*   DigOutInit() - Public function to initialize all digital outputs in OFF
*   state and supplying 5V. All configuration settings are stored in
*   doutConfigs structure.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void DigOutInit()
{
    /* Enable PORTA and PORTB clocks (all DOUT pins are in PORTA or PORTB). */
    SIM->SCGC5 |= SIM_SCGC5_PORTA(ENABLE);
    SIM->SCGC5 |= SIM_SCGC5_PORTB(ENABLE);

    /* Configure all DOUT pin MUXs to ALT 1 (GPIO), and configure all GPIO as
     * outputs. This is done for both the power and state pins for each DOUT,
     * with port name/gpio name/pin number defined in DOUT config structure.
     * Sets all DOUT pins to initial state/power, which is OFF/5V, as specified
     * in DOUT config structure. */
    PORTA->PCR[DOUT1_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT2_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT3_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT4_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT5_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT6_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT7_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT8_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);

    PORTB->PCR[DOUT1_STATE_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT2_STATE_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTB->PCR[DOUT3_STATE_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT4_STATE_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT5_STATE_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT6_STATE_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT7_STATE_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTA->PCR[DOUT8_STATE_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);

    GPIOA->PDDR |= GPIO_PDDR_PDD((OUTPUT << DOUT1_POWER_PIN_NUM) |
                                 (OUTPUT << DOUT2_POWER_PIN_NUM) |
                                 (OUTPUT << DOUT3_POWER_PIN_NUM) |
                                 (OUTPUT << DOUT4_POWER_PIN_NUM) |
                                 (OUTPUT << DOUT5_POWER_PIN_NUM) |
                                 (OUTPUT << DOUT6_POWER_PIN_NUM) |
                                 (OUTPUT << DOUT7_POWER_PIN_NUM) |
                                 (OUTPUT << DOUT8_POWER_PIN_NUM) |
                                 (OUTPUT << DOUT2_STATE_PIN_NUM) |
                                 (OUTPUT << DOUT4_STATE_PIN_NUM) |
                                 (OUTPUT << DOUT5_STATE_PIN_NUM) |
                                 (OUTPUT << DOUT6_STATE_PIN_NUM) |
                                 (OUTPUT << DOUT7_STATE_PIN_NUM) |
                                 (OUTPUT << DOUT8_STATE_PIN_NUM));

    GPIOB->PDDR |= GPIO_PDDR_PDD((OUTPUT << DOUT1_STATE_PIN_NUM) |
                                 (OUTPUT << DOUT3_STATE_PIN_NUM));
}

/******************************************************************************
*   DigOutSet() - Public function to set digital output states and powers via
*   requested message structure.
*
*   Parameters:
*
*       DigOutMsg_t msg - Message structure received from telemetry unit with
*       8 bit state field (msg.state_field) and 8 bit power field
*       (msg.power_field).
*
*   Return: None
******************************************************************************/
void DigOutSet(DigOutMsg_t msg)
{
    /* Convert state/power message to individual state/power bits for each
     * DOUT (i.e. msg.state_field bit 0 corresponds to DOUT1's desired state,
     * msg.state_field bit 1 corresponds to DOUT2's state, etc.) */
    DOUT1_STATE_SET(DOUT1_BIT(msg.state_field));
    DOUT2_STATE_SET(DOUT2_BIT(msg.state_field));
    DOUT3_STATE_SET(DOUT3_BIT(msg.state_field));
    DOUT4_STATE_SET(DOUT4_BIT(msg.state_field));
    DOUT5_STATE_SET(DOUT5_BIT(msg.state_field));
    DOUT6_STATE_SET(DOUT6_BIT(msg.state_field));
    DOUT7_STATE_SET(DOUT7_BIT(msg.state_field));
    DOUT8_STATE_SET(DOUT8_BIT(msg.state_field));

    DOUT1_POWER_SET(DOUT1_BIT(msg.power_field));
    DOUT2_POWER_SET(DOUT2_BIT(msg.power_field));
    DOUT3_POWER_SET(DOUT3_BIT(msg.power_field));
    DOUT4_POWER_SET(DOUT4_BIT(msg.power_field));
    DOUT5_POWER_SET(DOUT5_BIT(msg.power_field));
    DOUT6_POWER_SET(DOUT6_BIT(msg.power_field));
    DOUT7_POWER_SET(DOUT7_BIT(msg.power_field));
    DOUT8_POWER_SET(DOUT8_BIT(msg.power_field));
}
