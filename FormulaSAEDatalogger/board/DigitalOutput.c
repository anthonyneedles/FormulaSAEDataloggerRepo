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
*   Comments up to date as of: 05/22/2019
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

/* DOUT1 and DOUT3 are PORTB, all else are PORTA. */
#define DOUT1_STATE_PIN_NUM            1U
#define DOUT2_STATE_PIN_NUM           28U
#define DOUT3_STATE_PIN_NUM            0U
#define DOUT4_STATE_PIN_NUM           29U
#define DOUT5_STATE_PIN_NUM           27U
#define DOUT6_STATE_PIN_NUM           24U
#define DOUT7_STATE_PIN_NUM           26U
#define DOUT8_STATE_PIN_NUM           25U

/* PORTA. */
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

/* Sets DOUT state pin, turning on DOUT */
#define D1_ON (GPIOB->PDOR |= ((1U) << DOUT1_STATE_PIN_NUM))
#define D2_ON (GPIOA->PDOR |= ((1U) << DOUT2_STATE_PIN_NUM))
#define D3_ON (GPIOB->PDOR |= ((1U) << DOUT3_STATE_PIN_NUM))
#define D4_ON (GPIOA->PDOR |= ((1U) << DOUT4_STATE_PIN_NUM))
#define D5_ON (GPIOA->PDOR |= ((1U) << DOUT5_STATE_PIN_NUM))
#define D6_ON (GPIOA->PDOR |= ((1U) << DOUT6_STATE_PIN_NUM))
#define D7_ON (GPIOA->PDOR |= ((1U) << DOUT7_STATE_PIN_NUM))
#define D8_ON (GPIOA->PDOR |= ((1U) << DOUT8_STATE_PIN_NUM))

/* Clears DOUT state pin, turning off DOUT */
#define D1_OFF (GPIOB->PDOR &= ~((1U) << DOUT1_STATE_PIN_NUM))
#define D2_OFF (GPIOA->PDOR &= ~((1U) << DOUT2_STATE_PIN_NUM))
#define D3_OFF (GPIOB->PDOR &= ~((1U) << DOUT3_STATE_PIN_NUM))
#define D4_OFF (GPIOA->PDOR &= ~((1U) << DOUT4_STATE_PIN_NUM))
#define D5_OFF (GPIOA->PDOR &= ~((1U) << DOUT5_STATE_PIN_NUM))
#define D6_OFF (GPIOA->PDOR &= ~((1U) << DOUT6_STATE_PIN_NUM))
#define D7_OFF (GPIOA->PDOR &= ~((1U) << DOUT7_STATE_PIN_NUM))
#define D8_OFF (GPIOA->PDOR &= ~((1U) << DOUT8_STATE_PIN_NUM))

/* Sets DOUT power pin, selecting a 12V output on DOUT */
#define D1_12V (GPIOA->PDOR |= ((1U) << DOUT1_POWER_PIN_NUM))
#define D2_12V (GPIOA->PDOR |= ((1U) << DOUT2_POWER_PIN_NUM))
#define D3_12V (GPIOA->PDOR |= ((1U) << DOUT3_POWER_PIN_NUM))
#define D4_12V (GPIOA->PDOR |= ((1U) << DOUT4_POWER_PIN_NUM))
#define D5_12V (GPIOA->PDOR |= ((1U) << DOUT5_POWER_PIN_NUM))
#define D6_12V (GPIOA->PDOR |= ((1U) << DOUT6_POWER_PIN_NUM))
#define D7_12V (GPIOA->PDOR |= ((1U) << DOUT7_POWER_PIN_NUM))
#define D8_12V (GPIOA->PDOR |= ((1U) << DOUT8_POWER_PIN_NUM))

/* Clears DOUT power pin, selecting a 5V output on DOUT */
#define D1_5V (GPIOA->PDOR &= ~((1U) << DOUT1_POWER_PIN_NUM))
#define D2_5V (GPIOA->PDOR &= ~((1U) << DOUT2_POWER_PIN_NUM))
#define D3_5V (GPIOA->PDOR &= ~((1U) << DOUT3_POWER_PIN_NUM))
#define D4_5V (GPIOA->PDOR &= ~((1U) << DOUT4_POWER_PIN_NUM))
#define D5_5V (GPIOA->PDOR &= ~((1U) << DOUT5_POWER_PIN_NUM))
#define D6_5V (GPIOA->PDOR &= ~((1U) << DOUT6_POWER_PIN_NUM))
#define D7_5V (GPIOA->PDOR &= ~((1U) << DOUT7_POWER_PIN_NUM))
#define D8_5V (GPIOA->PDOR &= ~((1U) << DOUT8_POWER_PIN_NUM))

/* Macros accepting 8 bit field to find and isolate DOUT's relevant bit and
 * shift over to bit 0 position for relevant SET function */
#define D1_BIT(x) (((x) >> DOUT1_BIT_NUM) & BIT_0_MASK)
#define D2_BIT(x) (((x) >> DOUT2_BIT_NUM) & BIT_0_MASK)
#define D3_BIT(x) (((x) >> DOUT3_BIT_NUM) & BIT_0_MASK)
#define D4_BIT(x) (((x) >> DOUT4_BIT_NUM) & BIT_0_MASK)
#define D5_BIT(x) (((x) >> DOUT5_BIT_NUM) & BIT_0_MASK)
#define D6_BIT(x) (((x) >> DOUT6_BIT_NUM) & BIT_0_MASK)
#define D7_BIT(x) (((x) >> DOUT7_BIT_NUM) & BIT_0_MASK)
#define D8_BIT(x) (((x) >> DOUT8_BIT_NUM) & BIT_0_MASK)

/******************************************************************************
*   DOutInit() - Public function to initialize all digital outputs in OFF
*   state and supplying 5V. All configuration settings are stored in
*   doutConfigs structure.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void DOutInit()
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA(ENABLE);
    SIM->SCGC5 |= SIM_SCGC5_PORTB(ENABLE);

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
    D1_OFF;
    D2_OFF;
    D3_OFF;
    D4_OFF;
    D5_OFF;
    D6_OFF;
    D7_OFF;
    D8_OFF;
    D1_12V;
    D2_12V;
    D3_12V;
    D4_12V;
    D5_12V;
    D6_12V;
    D7_12V;
    D8_12V;
}

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
*
*   Author: Anthony Needles
******************************************************************************/
void DOutSet(dout_msg_t msg)
{
    /* Convert state/power message to individual state bits for each DOUT and
     * enables/disables output accordingly */
    (D1_BIT(msg.state_field) == 1U) ? D1_ON : D1_OFF;
    (D2_BIT(msg.state_field) == 1U) ? D2_ON : D2_OFF;
    (D3_BIT(msg.state_field) == 1U) ? D3_ON : D3_OFF;
    (D4_BIT(msg.state_field) == 1U) ? D4_ON : D4_OFF;
    (D5_BIT(msg.state_field) == 1U) ? D5_ON : D5_OFF;
    (D6_BIT(msg.state_field) == 1U) ? D6_ON : D6_OFF;
    (D7_BIT(msg.state_field) == 1U) ? D7_ON : D7_OFF;
    (D8_BIT(msg.state_field) == 1U) ? D8_ON : D8_OFF;

    /* Convert state/power message to individual power bits for each DOUT and
     * changes power output accordingly */
    (D1_BIT(msg.power_field) == 1U) ? D1_12V : D1_5V;
    (D2_BIT(msg.power_field) == 1U) ? D2_12V : D2_5V;
    (D3_BIT(msg.power_field) == 1U) ? D3_12V : D3_5V;
    (D4_BIT(msg.power_field) == 1U) ? D4_12V : D4_5V;
    (D5_BIT(msg.power_field) == 1U) ? D5_12V : D5_5V;
    (D6_BIT(msg.power_field) == 1U) ? D6_12V : D6_5V;
    (D7_BIT(msg.power_field) == 1U) ? D7_12V : D7_5V;
    (D8_BIT(msg.power_field) == 1U) ? D8_12V : D8_5V;
}
